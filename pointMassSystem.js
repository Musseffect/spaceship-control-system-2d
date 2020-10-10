class Particle{
    //mass;
    //localPosition;
    constructor(mass,localPosition){
        this.mass = mass;
        this.localPosition = localPosition;
    }
}

class RigitParticleSystem{
    /*particles = [];
    totalMass;
    momentOfInertia;
    referencePoint;
    position = new vec2(0.0,0.0);
    velocity = new vec2(0.0,0.0);
    angle = 0.0;
    angularVelocity = 0.0;
    forces = [];*/
    constructor(particles,referencePoint){
        this.particles = particles.splice();
        this.referencePoint = referencePoint;  
        this.position = new vec2(0.0,0.0);
        this.velocity = new vec2(0.0,0.0); 
        this.angle = 0.0;
        this.angularVelocity = 0.0;
        this.forces = [];
        this.computeMassAndInertia();
    }
    computeMassAndInertia(){
        this.totalMass = 0;
        this.momentOfInertia = 0.0;
        this.particles.forEach(function(item){
            this.totalMass+=item.mass;
            this.momentOfInertia += item.mass*item.localPosition.lengthSqr();
        });
    }
    clearForces(){
        this.forces = [];
    }
    addForce(force,localPosition){
        this.forces.push({force:force,position:localPosition});
    }
}