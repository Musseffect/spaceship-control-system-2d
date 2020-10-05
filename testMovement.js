function mod(x,n){
	return x-n*Math.floor(x/n);
}

let TestMovementMain = (function(){
	let body = new RigidBody2D(40,10,1,new vec2(300,300),
	new vec2(0,0),0,0);
	let time = undefined;
	let start = undefined;
	function reset(){
		body = new RigidBody2D(40,10,1,new vec2(300,300),
	new vec2(0,0),0,0);
		frames = 0;
	}
	let canvas = document.getElementById("canvas");
	let context = canvas.getContext("2d");
	let frames = 0;
	function frame(timestamp){
		body.resetForces();
		if(!time){
			time = timestamp;
			start = time;
		}
		context.setTransform(1, 0, 0, 1, 0, 0);
		let dt = timestamp - time;
		time = timestamp;
		if(frames >= 1&&frames<=100){	
			/*body.applyForce(new vec2(-0,10.),new vec2(20.,-10.));
			body.applyForce(new vec2(-0,-10.),new vec2(-20.,10.));*/
		    body.applyForce(new vec2(-0,10.),new vec2(-20.,-10.));
			body.applyForce(new vec2(-0,10.),new vec2(20.,-10.));
		    body.applyForce(new vec2(10,0.),new vec2(-20.,10.));
			body.applyForce(new vec2(10,0.),new vec2(-20.,-10.));
		}
		if(frames>=301&&frames<=350){
            /*body.applyForce(new vec2(-0,-2*10.),new vec2(20.,10.));
            body.applyForce(new vec2(-0,2*10.),new vec2(-20.,-10.));*/
            body.applyForce(new vec2(-0,-2*10.),new vec2(-20.,10.));
            body.applyForce(new vec2(-0,-2*10.),new vec2(20.,10.));
            body.applyForce(new vec2(-20,0.),new vec2(20.,10.));
            body.applyForce(new vec2(-20,0.),new vec2(20.,-10.));
            //console.log("stop");
        }
		body.integrate(dt*0.001);
		body.position.x = mod(body.position.x,600);
		body.position.y = mod(body.position.y,600);
		context.clearRect(0, 0, this.canvas.width, this.canvas.height);
		body.draw(context);
		frames++;
		window.requestAnimationFrame(frame);
	}
	return {
		run:function(){
			window.requestAnimationFrame(frame);
		},
		reset:reset
	}
})();