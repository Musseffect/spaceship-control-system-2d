class RigidBody2D{

	constructor(w,h,mass,pos,vel,ang,angVel){
		this.width = w;
		this.height = h;
		this.mass = mass;
		this.computeInertia();

		this.position = pos;
		this.velocity = vel;

		this.angle = ang;
		this.angularVelocity = angVel;

		this.forces = [];
	}
	draw(ctx){
        ctx.save();
        /*ctx.translate(ctx.canvas.width, 0);
        ctx.scale(-1, 1);*/

		ctx.translate(this.position.x, this.position.y);
		ctx.rotate(this.angle);
		ctx.translate(-this.position.x, -this.position.y);
		ctx.fillStyle = "#FFFFFFFF";
        ctx.fillRect(this.position.x-this.width*0.5, this.position.y-this.height*0.5, this.width, this.height);
        ctx.fillStyle = "#FF0000";
        ctx.fillRect(this.position.x+this.width*0.1, this.position.y-this.height*0.3, this.width*0.3, this.height*0.6);
        ctx.fillStyle = "#FFFF0088";
        ctx.translate(this.position.x, this.position.y);
        this.forces.forEach(function(item){
            let center = item.r;
            let force = item.force;
            let l = force.length();
            if(l<0.00001){
                return;
			}
			l = Math.max(l,0.3);
            let a = force.angle();
            let b = 1.-Math.exp(-l);
            const triangleDist = 15;
            ctx.translate(center.x, center.y);
            ctx.rotate(-a);
            ctx.beginPath();
            ctx.moveTo(0, 0);
            ctx.lineTo(triangleDist*b, triangleDist*b*0.5);
            ctx.lineTo(triangleDist*b, -triangleDist*b*0.5);
            ctx.fill();
            ctx.rotate(a);
            ctx.translate(-center.x, -center.y);
        });
        ctx.translate(-this.position.x, -this.position.y);
        ctx.restore();
	}
	projectWtoL(p){
		let l = vec2.sub(p,this.position);
        let x = new vec2(Math.cos(this.angle),Math.sin(this.angle));
        let y = new vec2(-Math.sin(this.angle),Math.cos(this.angle));
        return new vec2(vec2.dot(l,x),vec2.dot(l,y));
	}
	projectVecWtoL(p){
        let x = new vec2(Math.cos(this.angle),Math.sin(this.angle));
        let y = new vec2(-Math.sin(this.angle),Math.cos(this.angle));
        return new vec2(vec2.dot(p,x),vec2.dot(p,y));
	}
	projectLtoW(p){
        let x = new vec2(Math.cos(-this.angle),Math.sin(-this.angle));
        let y = new vec2(-Math.sin(-this.angle),Math.cos(-this.angle));
		return (new vec2(vec2.dot(p,x),vec2.dot(p,y))).addSelf(this.position);
	}
	computeInertia(){
		this.intertia = 1/12*this.mass*(Math.pow(this.width,2)+Math.pow(this.height,2));
	}
	setMass(mass){
		this.mass = mass;
		this.computeInertia();
	}
	setSize(w,h){
		this.width = w;
		this.height = h;
		this.computeInertia();
	}
	integrate(dt){
		this.position.addSelf(vec2.scale(this.velocity,dt));
		let totalForce = new vec2(0,0);
		let totalTorque = 0;
		this.forces.forEach(function(item){
			totalForce.addSelf(item.force);
			totalTorque+=vec2.cross(item.r,item.force);
		});
        let x = new vec2(Math.cos(-this.angle),Math.sin(-this.angle));
        let y = new vec2(-Math.sin(-this.angle),Math.cos(-this.angle));
        totalForce = new vec2(vec2.dot(totalForce,x),vec2.dot(totalForce,y));//total force in global coordinates
		let acceleration = vec2.scale(totalForce,1./this.mass);
		this.velocity.addSelf(vec2.scale(acceleration,dt));
		this.position.addSelf(vec2.scale(acceleration,dt*dt/2.));
		
		this.angle+=this.angularVelocity*dt;
		let angAcceleration = totalTorque/this.intertia;
		this.angularVelocity+=angAcceleration*dt;
		this.angle+=angAcceleration*dt*dt/2;
	}
	resetForces(){
		this.forces = [];
	}
	applyForce(force,localPosition){
		this.forces.push({force:force,r:localPosition});
	}
}