class vector{
	data;
	constructor(data){
		this.data = data;
    }
    copy(){
        return new vector(this.data.slice());
    }
	static empty(length){
		let data;
        (data = []).length = length; 
        data.fill(0);
        return new vector(data);
	}
    length(){
        return this.data.length;
    }
    static dot(a, b) {
        let result = 0;
        for(let i = 0;i<a.length();i++){
            result+=a.data[i]*b.data[i];
        }
        return result;
    }
    static empty(length){
        let data;
        (data = []).length = length; 
        data.fill(0);
        return new vector(data);
    }
    static add(a,b){
        let result = [];
        for(let i=0;i<a.data.length;i++)
            result.push(a.data[i] + b.data[i]);
        return new vector(result);
    }
    static sub(a,b){
        let result = [];
        for(let i=0;i<a.data.length;i++)
            result.push(a.data[i] - b.data[i]);
        return new vector(result);
    }
    static mult(a,b){
        let result = [];
        for(let i=0;i<a.data.length;i++)
            result.push(a.data[i] * b.data[i]);
        return new vector(result);
    }
    static scale(a,b){
        let result = [];
        for(let i=0;i<a.data.length;i++)
            result.push(a.data[i] * b);
        return new vector(result);
    }
    clone(){
        return new vector(this.data.slice());
    }
    addSelf(b){
        for(let i=0;i<this.data.length;i++)
            this.data[i]+=b.data[i];
        return this;
    }
    subSelf(b){
        for(let i=0;i<this.data.length;i++)
            this.data[i]-=b.data[i];
        return this;
    }
    scaleSelf(b){
        for(let i=0;i<this.data.length;i++)
            this.data[i]*=b;
        return this;
    }
    get(i){
        return this.data[i];
    }
    set(i, value){
        this.data[i] = value;
    }
    length(){
        return this.data.length;
    }
    getSubVector(offset,length){
        let resultData = new Array(length);
        for(let i=0;i<length;i++)
            resultData[i] = this.data[offset+i];
        return new vector(resultData);
    }
    addSubVector(b, offset){
        for(let i=0;i<b.length();i++)
            this.data[i+offset] += b.get(i);
        return this;
    }
    subSubVector(b, offset){
        for(let i=0;i<b.length();i++)
            this.data[i+offset] -= b.get(i);
        return this;
    }
    add(b, dest){
        if(dest==undefined){
            dest = this;
        }
        for(let i=0;i<this.data.length;i++)
            dest.data[i] = this.data[i] + b.data[i];
        return dest;
    }
    sub(b, dest){
        if(dest==undefined){
            dest = this;
        }
        for(let i=0;i<this.data.length;i++)
            dest.data[i] = this.data[i] - b.data[i];
        return dest;
    }
    mult(b, dest){
        if(dest==undefined){
            dest = this;
        }
        for(let i=0;i<this.data.length;i++)
            dest.data[i] = this.data[i] * b.data[i];
        return dest;
    }
    scale(b, dest){
        if(dest==undefined){
            dest = this;
        }
        for(let i=0;i<this.data.length;i++)
            dest.data[i] = this.data[i] * b;
        return dest;
    }
    norm2(){
        let result = 0;
        for(let i=0;i<this.data.length;i++)
            result += this.data[i]*this.data[i];
        return Math.sqrt(result);
    }
    clamp(min,max){
        for(let i = 0;i<this.data.length;i++){
            this.data[i] = clamp(this.data[i],min.data[i],max.data[i]);
        }
    }
    print(fractionDigits){
        if(!fractionDigits)
            fractionDigits = 4;
        let result = "[ ";
        this.data.forEach((item,index)=>result+=item.toFixed(fractionDigits) +" ");
        return result + "]";
    }
}

class matrix{
    w;
    h;
    data;
    constructor(data,w,h){
        this.data = data;
        this.w = w;
        this.h = h;
    }
    static empty(w,h){
        let data;
        (data = []).length = w*h; 
        data.fill(0);
        return new matrix(data,w,h);
    }
    copy(){
        return new matrix(this.data.slice(),this.w,this.h);
    }
    static identity(size){
        let data;
        (data = []).length = size*size; 
        data.fill(0);
        for(let i=0;i<size;i++){
            data[i+i*size] = 1;
        }
        return new matrix(data,size,size);
    }
    static mult(a,b){
        let result = matrix.empty(b.w,a.h);
        //for each cell in the result
        for(let j=0;j<a.h;j++){
            for(let i=0;i<b.w;i++){
                let value = 0;
                for(let k = 0;k<a.w;k++){
                    value += a.get(j,k) * b.get(k,i);
                }
                result.set(value,j,i);
            }
        }
        return result;
    }
    static multVec(a,b){
        let result = vector.empty(a.h);
        for(let j=0;j<a.h;j++){
            let v = 0;
            for(let i=0;i<a.w;i++)
            {
                v+=a.get(j,i) * b.get(i);
            }
            result.set(j,v);
        }
        return result;
    }    
    get(row,column){
        return this.data[row*this.w+column];
    }
    set(value,row,column){
        this.data[row*this.w+column] = value;
    }
    transpose(){
        let result = [];
        for(let i=0;i<this.w;i++){
            for(let j=0;j<this.h;j++){
                result.push(this.data[i + j*this.w]);
            }
        }
        return new matrix(result,this.h,this.w);
    }
    static solve(A,b){
        var rang=b.length();
        var x=vector.empty(rang);
        let epsilon=0.001
        var indexes = new Array(rang);
        for (var i = 0; i < rang; i++){
            indexes[i] = i;
        }
        for (var l = 0; l < rang; l++){
            var max = l;
            for (var i = l + 1; i < rang; i++){
                if (Math.abs(A.get(indexes[i],l))>Math.abs(A.get(indexes[max],l)))
                    max = i;
            }
            if (max != l){
                var temp = indexes[l];
                indexes[l] = indexes[max];
                indexes[max] = temp;
            }
            if (Math.abs(A.get(indexes[l],l)) < epsilon){
                for(var i=0;i<rang;i++)
                    x.set(i,0.0);
                return x;
            }
            for (var i = l + 1; i < rang; i++)
                A.set(A.get(indexes[l],i) / A.get(indexes[l],l),indexes[l],i);
            b.set(indexes[l],b.get(indexes[l]) / A.get(indexes[l],l));
            A.set(1,indexes[l],l);
    
            for (var i = l + 1; i < rang; i++){
                for (var k = l + 1; k < rang; k++)
                    A.set(A.get(indexes[i],k) - A.get(indexes[i],l) * A.get(indexes[l],k),indexes[i],k);
                b.set(indexes[i],b.get(indexes[i]) - A.get(indexes[i],l) * b.get(indexes[l]));
                A.set(0,indexes[i],l);
            }
        }
        x.set(rang - 1,b.get(indexes[rang - 1]));
        for (var i = rang - 2; i > -1; i--){
            var k = 0.;
            for (var j = i + 1; j < rang; j++){
                k = k + A.get(indexes[i],j) * x.get(j);
            }
            x.set(i,b.get(indexes[i]) - k);
        }
        return x;
    }
    inverse(){
        if(this.w!=this.h)
            throw new Error("Non square matrix");
        let result = this.copy();
        for(let i = 0;i<this.w;i++){
            let v = vector.empty(this.w);
            v.set(i,1);
            let column = matrix.solve(this.copy(),v);
            for(let j = 0;j<this.h;j++){
                result.set(column.get(j),j,i);
            }
        }
        return result;
    }
    print(fractionDigits){
        if(!fractionDigits)
            fractionDigits = 4;
        let result = "";
        for(let j=0;j<this.h;j++){
            result += j>0?"\n| ":"| ";
            for(let i=0;i<this.w;i++)
                result+=this.data[i+j*this.w].toFixed(fractionDigits)+" "
            result += "|";
        }
        return result;
    }
}