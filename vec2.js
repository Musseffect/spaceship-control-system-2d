class vec2{
	//x;
	//y;
	constructor(x,y){
		this.x = x;
		this.y = y;
    }
    angle(){
        return Math.atan2(this.y,-this.x);
    }
	addSelf(b){
		this.x+=b.x;
		this.y+=b.y;
		return this;
	}
	subSelf(b){
		this.x-=b.x;
		this.y-=b.y;
		return this;
	}
	static sub(a,b){
		return new vec2(a.x-b.x,a.y-b.y);
	}
	static add(a,b){
		return new vec2(a.x+b.x,a.y+b.y);
	}
	static dot(a,b){
		return a.x*b.x+a.y*b.y;
	}
	static cross(a,b){
		return a.x*b.y-a.y*b.x;
	}
	static mul(a,b){
		return new vec2(a.x*b.x,a.y*b.y)
	}
	lengthSqr(){
		return vec2.dot(this,this);
	}
	length(){
		return Math.sqrt(this.lengthSqr());
	}
	static dist(a,b){
		return Math.sqrt(Math.pow(a.x-b.x,2)+Math.pow(a.y-b.y,2));
	}
	static scale(a,l){
		return new vec2(a.x*l,a.y*l);
	}
}