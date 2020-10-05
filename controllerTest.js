class StateSpaceController{
    K;
    constructor(K){
        this.K = K;
    }
    computeInput(err){
        return matrix.multVec(this.K,err);
    }
}


class PID{
    Kp;
    Ki;
    Kd;
    prevError;
    errorIntegral;
    constructor(Kp,Ki,Kd){
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.errorIntegral = vector.empty(this.Kp.length());
        this.prevError = vector.empty(this.Kp.length());
    }
    reset(){
        this.errorIntegral = vector.empty(this.Kp.length());
        this.prevError = vector.empty(this.Kp.length());
    }
    computeInput(err,dt){
        let derivative = vector.scale(vector.sub(err,this.prevError),1./Math.max(dt,0.0001));
        for(let i=0;i<err.length();i++){
            if(Math.sign(this.prevError.get(i))!=Math.sign(err.get(i))){
                this.errorIntegral.set(i,0.);
            }
        }
        this.errorIntegral.addSelf(err);
        let result = derivative.mult(this.Kd)
            .addSelf(vector.mult(err,this.Kp))
            .addSelf(vector.mult(this.errorIntegral,this.Ki));
        this.prevError = err;
        return result;
    }
}


let PIDTestMain = (function(){
    let path = [
        {position:new vec2(0.5,0.5),angle:0,time:1},
        {position:new vec2(0.75,0.75),angle:0,time:8},
        {position:new vec2(0.75,0.75),angle:Math.PI/2,time:8},
        {position:new vec2(0.75,0.25),angle:Math.PI/2,time:8},
        {position:new vec2(0.75,0.25),angle:0,time:8},
        {position:new vec2(0.25,0.25),angle:0,time:8},
        {position:new vec2(0.25,0.25),angle:Math.PI/2,time:8},
        {position:new vec2(0.25,0.75),angle:Math.PI/2,time:8},
        {position:new vec2(0.25,0.75),angle:0,time:7},
        {position:new vec2(0.75,0.75),angle:0,time:6},
        {position:new vec2(0.5,0.5),angle:Math.PI/2,time:5},
        {position:new vec2(0.5,0.5),angle:0,time:10e8}
    ]
    const w = 40;
    const h = 10;
    let currentVertex;
    let integral;
    let prevError;
    let body;
    let time;
    let localTime;
    let Kp = {p:20.5,i:1.*5.5,d:14.01};
    let Ka = {p:0.851*5.5,i:0.7*5.5,d:125.5};
    let goalTimer;
	let canvas = document.getElementById("canvas");
    let context = canvas.getContext("2d");
    let pid = new PID(
        new vector([20.5,20.5,14.0*5.5]),
        new vector([0.01*5.5,0.01*5.5,0.0*5.5]),
        new vector([14.01,14.01,225.5])
    );
    let ssc = new StateSpaceController(new matrix([
        42,55.5,0,0,0,0,
        0,0,42,55.5,0,0,
        0,0,0,0,152,225.5
    ],6,3));
    function reset(){
        currentVertex = 0;
        localTime = 0;
        goalTimer = 0;
        time = undefined;
        integral = {position:new vec2(0.,0.),angle:0};
        prevError = {position:new vec2(0.,0.),angle:0};
        body = new RigidBody2D(w,h,1,new vec2(300,300),
        new vec2(0,0),0,0);
    }
    function getGoal(){
        /*return {
            position:new vec2(Math.cos(time*0.001)*200.0+Math.sin(time*0.01)*1+300,Math.sin(time*0.001)*200.0+300),
            angle:time*0.001%(Math.PI*2)
            };*/
        while(goalTimer>path[currentVertex].time&&currentVertex<path.length-1){
            goalTimer-=path[currentVertex].time;
            currentVertex++;
            path[currentVertex].position.x = Math.random();
            path[currentVertex].position.y = Math.random();
            path[currentVertex].angle = Math.random()*Math.PI*2;
        }
        let vertex = path[currentVertex];
        return {position:vec2.mul(vertex.position,new vec2(600,600)),angle:vertex.angle};
    }
    /*function frame(timestamp){
        if(!time){
            time = timestamp;
        }
        let dt = (timestamp - time)*0.001;
        localTime+=dt;
        time = timestamp;
        goalTimer+=dt;
        body.integrate(dt);	
        let goal = getGoal();
        goal = {
            position:new vec2(Math.cos(time*0.001)*200.0+Math.sin(time*0.01)*1+300,Math.sin(time*0.001)*200.0+300),
            angle:time*0.001%(Math.PI*2)};
        
        //project goal position 
        let projectedPosition = vec2.sub(goal.position,body.position);
        let x = new vec2(Math.cos(body.angle),Math.sin(body.angle));
        let y = new vec2(-Math.sin(body.angle),Math.cos(body.angle));
        let _x = vec2.dot(projectedPosition,x);
        let _y = vec2.dot(projectedPosition,y);
        projectedPosition = new vec2(_x,_y);
        //console.log(projectedPosition);
        let currentError = {
            position:projectedPosition,
            angle: Math.atan2(Math.sin(goal.angle-body.angle), Math.cos(goal.angle-body.angle))
        };
        if(Math.sign(prevError.angle)!=Math.sign(currentError.angle))
            integral.angle = 0;
        if(Math.sign(prevError.position.x)!=Math.sign(currentError.position.x))
            integral.position.x = 0;
        if(Math.sign(prevError.position.y)!=Math.sign(currentError.position.y))
            integral.position.y = 0;
        integral = {
            position:integral.position.addSelf(vec2.scale(currentError.position,dt)),
            angle:integral.angle + currentError.angle*dt,
            angularVelocity:integral.angularVelocity + currentError.angularVelocity*dt
        };
        integral.position.x = Math.min(Math.abs(integral.position.x),125.1)*Math.sign(integral.position.x);
        integral.position.y = Math.min(Math.abs(integral.position.y),125.1)*Math.sign(integral.position.y);
        integral.angle = Math.min(Math.abs(integral.angle),1.5)*Math.sign(integral.angle);
        //console.log(`${currentError.angle} - body: ${body.angle}`);
        //console.log(integral);
        if(dt!=0){
            let derivative = {
                position:vec2.scale(vec2.sub(currentError.position,prevError.position),1./Math.max(dt,0.0001)),
                angle:(currentError.angle - prevError.angle)/Math.max(dt,0.0001)
            };
            body.resetForces();
            let force = vec2.scale(currentError.position,Kp.p).addSelf(vec2.scale(integral.position,Kp.i)).addSelf(vec2.scale(derivative.position,Kp.d));
            let torque = Ka.p*currentError.angle+Ka.i*integral.angle+Ka.d*derivative.angle;
            let l = Math.min(force.length(),1000)/Math.max(force.length(),0.01);
            torque = Math.min(Math.abs(torque),44500)*Math.sign(torque);
            force = new vec2(force.x*l,force.y*l);
            //body.applyForce(new vec2(140.*Math.sin(body.angle),140.*Math.cos(body.angle)),new vec2(0,0));
            if(force.x>0){
                body.applyForce(new vec2(force.x/2,0.),new vec2(-w/2,h/2));
                body.applyForce(new vec2(force.x/2,0.),new vec2(-w/2,-h/2));
            }else{
                body.applyForce(new vec2(force.x/2,0.),new vec2(w/2,h/2));
                body.applyForce(new vec2(force.x/2,0.),new vec2(w/2,-h/2));
            }
            if(force.y>0){
                body.applyForce(new vec2(0.,force.y/2),new vec2(w/2,-h/2));
                body.applyForce(new vec2(0.,force.y/2),new vec2(-w/2,-h/2));
            }else{
                body.applyForce(new vec2(0.,force.y/2),new vec2(w/2,h/2));
                body.applyForce(new vec2(0.,force.y/2),new vec2(-w/2,h/2));
            }
            const dist = Math.sqrt(w*w/4+h*h/4);
            //torque*=0.1;
            if(torque>0){
                body.applyForce(new vec2(0.,torque/dist*0.5),new vec2(w/2,-h/2));
                body.applyForce(new vec2(0.,-torque/dist*0.5),new vec2(-w/2,h/2));
            }
            else{
                body.applyForce(new vec2(0.,torque/dist*0.5),new vec2(w/2,h/2));
                body.applyForce(new vec2(0.,-torque/dist*0.5),new vec2(-w/2,-h/2));
            }
        }
		drawGoal(goal);
		body.draw(context);
        prevError = currentError;
        window.requestAnimationFrame(frame);
    }*/
    function drawGoal(goal){
		context.clearRect(0, 0, canvas.width, canvas.height);
        context.fillStyle = "#000000FF";
        context.fillRect(0, 0, canvas.width, canvas.height);
        context.fillStyle = "#00FF00FF";
        context.fillRect(goal.position.x-10, goal.position.y-10, 20, 20);
        
        context.save();
        
        context.translate(goal.position.x,goal.position.y);
        context.rotate(goal.angle);
        context.translate(-goal.position.x,-goal.position.y);
        context.fillRect(goal.position.x, goal.position.y-2, 25, 4);
        
        context.restore();
    }
    function applyInputManual(body,input){
        let force = new vec2(input.get(0),input.get(1));
        let torque = input.get(2);            
        let l = Math.min(force.length(),1000)/Math.max(force.length(),0.01);
        torque = Math.min(Math.abs(torque),44500)*Math.sign(torque);
        force = new vec2(force.x*l,force.y*l);
        //body.applyForce(new vec2(140.*Math.sin(body.angle),140.*Math.cos(body.angle)),new vec2(0,0));
        if(force.x>0){
            body.applyForce(new vec2(force.x/2,0.),new vec2(-w/2,h/2));
            body.applyForce(new vec2(force.x/2,0.),new vec2(-w/2,-h/2));
        }else{
            body.applyForce(new vec2(force.x/2,0.),new vec2(w/2,h/2));
            body.applyForce(new vec2(force.x/2,0.),new vec2(w/2,-h/2));
        }
        if(force.y>0){
            body.applyForce(new vec2(0.,force.y/2),new vec2(w/2,-h/2));
            body.applyForce(new vec2(0.,force.y/2),new vec2(-w/2,-h/2));
        }else{
            body.applyForce(new vec2(0.,force.y/2),new vec2(w/2,h/2));
            body.applyForce(new vec2(0.,force.y/2),new vec2(-w/2,h/2));
        }
        const dist = Math.sqrt(w*w/4+h*h/4);
        //torque*=0.1;
        if(torque>0){
            body.applyForce(new vec2(0.,torque/dist*0.5),new vec2(w/2,-h/2));
            body.applyForce(new vec2(0.,-torque/dist*0.5),new vec2(-w/2,h/2));
        }
        else{
            body.applyForce(new vec2(0.,torque/dist*0.5),new vec2(w/2,h/2));
            body.applyForce(new vec2(0.,-torque/dist*0.5),new vec2(-w/2,-h/2));
        }
    }
    function applyInputAuto(body,input){
        const engines = [
            {maxForce: 2800, position:new vec2(-w/2,-h/2),forceDirection:new vec2(0,1.),priority:1},
            {maxForce: 2800, position:new vec2(w/2,-h/2),forceDirection:new vec2(0,1.),priority:1},
            {maxForce: 2800, position:new vec2(-w/2,h/2),forceDirection:new vec2(0,-1.),priority:1},
            {maxForce: 2800, position:new vec2(w/2,h/2),forceDirection:new vec2(0,-1.),priority:1},
            {maxForce: 43600, position:new vec2(-w/2,0),forceDirection:new vec2(1,0.),priority:2},
            {maxForce: 43600, position:new vec2(w/2,0),forceDirection:new vec2(-1,0.),priority:2},
            {maxForce: 1600, position:new vec2(w/2,h/2),forceDirection:new vec2(-1,0.),priority:2},
            {maxForce: 1600, position:new vec2(w/2,-h/2),forceDirection:new vec2(-1,0.),priority:2},
            {maxForce: 1600, position:new vec2(-w/2,h/2),forceDirection:new vec2(1,0.),priority:2},
            {maxForce: 1600, position:new vec2(-w/2,-h/2),forceDirection:new vec2(1,0.),priority:2}
        ];
        let aArray = [];
        for(let i=0;i<engines.length;i++){
            aArray.push(engines[i].forceDirection.x);
        }
        for(let i=0;i<engines.length;i++){
            aArray.push(engines[i].forceDirection.y);
        }
        for(let i=0;i<engines.length;i++){
            aArray.push(vec2.cross(engines[i].position,engines[i].forceDirection));
        }
        /*for(let i=0;i<engines.length;i++){
            aArray.push(engines[i].priority);
        }*/
        let A = new matrix(aArray,engines.length,3);
        let AT = A.transpose();
        let B = new vector([input.get(0),input.get(1),input.get(2)]);
        let AAT = matrix.mult(A,AT);
        let x = matrix.multVec(matrix.mult(AT,AAT.inverse()),B);
        /*let BAT = matrix.multVec(AT,B);
        let x = matrix.solve(AAT,BAT);*/
        let scale = 1.;
        for(let i=0;i<engines.length;i++){
            scale = Math.max(scale,x.get(i)/engines[i].maxForce);
            x.set(i,Math.max(0,x.get(i)));
        }
        x.scaleSelf(1./scale);
        for(let i=0;i<engines.length;i++){
            body.applyForce(vec2.scale(engines[i].forceDirection,x.get(i)),engines[i].position);
        }
        //body.applyForce(new vec2(140.*Math.sin(body.angle),140.*Math.cos(body.angle)),new vec2(0,0));
        
    }
    function frame(timestamp){
        if(!time){
            time = timestamp;
            window.requestAnimationFrame(frame);
            return;
        }
        
        let dt = (timestamp - time)*0.001;
        localTime+=dt;
        goalTimer+=dt;

        let goal = getGoal();
        
        drawGoal(goal);
        body.draw(context);

        let positionError = body.projectWtoL(goal.position);
        let velocityError = body.projectVecWtoL(vec2.sub(new vec2(0.,0.),body.velocity));
        let angleError = Math.atan2(Math.sin(goal.angle-body.angle), Math.cos(goal.angle-body.angle));
        let angularVelocityError = -body.angularVelocity;
        console.log(angleError)
        let input = pid.computeInput(
            new vector([positionError.x,positionError.y, angleError]),
            dt);//values of required input force and torque in local coordinates
        
        input = ssc.computeInput(
            new vector([positionError.x,velocityError.x,positionError.y,velocityError.y,angleError,angularVelocityError]));
        body.resetForces();
        //applyInputManual(body,input);
        applyInputAuto(body,input);
        body.integrate(dt);	

        time = timestamp;
        window.requestAnimationFrame(frame);
    }



    return {
        run:function(){
            reset();
            window.requestAnimationFrame(frame);
        },
        reset:reset
    };
})();

