
////////predefined 
var knotvec_circ = [0,0,0,0.25,0.25,0.5,0.5,0.75,0.75,1,1,1]; //knotvec_circ and ptsweights_circ hold true for ellipses
var ptsweights_circ = [1, 0.70710678118, 1, 0.70710678118, 1, 0.70710678118,1,0.70710678118,1];
////////end predefined

//all body shapes are represented with NURBS -> http://en.wikipedia.org/wiki/Non-uniform_rational_B-spline

//memory variables
var isGravityOn = false;
var correction_constant = 0.5
var threshold_constant = 5;


var Material = function(e,mue){
	this.e = e;
	this.mue = mue;
}


var Gravity =function(){
	this.acc = 0;
	this.acc_def = "space";
	this.isGravityOn = false;
	this.on = function(phys_obj_array,planet){
		if(planet=="earth"){
			this.acc = -9.81;
			this.acc_def = "earth";
			isGravityOn
		}else if(planet=="moon"){
			this.acc = -1.622;
			this.acc_def = "moon";
		}else if(!isNaN(planet)){
			this.acc = planet;
			this.acc_def = "custom";
		}
		if(!isGravityOn || isGravityOn === undefined){
			for (var i = 0; i < phys_obj_array.length; i++) {
				if(!(phys_obj_array[i] instanceof Environment)){
					(phys_obj_array[i]).acc.x2 =(phys_obj_array[i]).acc.x2 +this.acc;
					this.isGravityOn = true;
				}
			}
		}
	}
	this.off = function(phys_obj_array){
		if(isGravityOn){
			for (var i = 0; i < phys_obj_array.length; i++) {
				(phys_obj_array[i]).acc.x2 =(phys_obj_array[i]).acc.x2 -this.acc;
				this.isGravityOn = false;
				this.acc_def = "space";
			}
		}
	}
	this.modify_gravity = function(obj,val){
		obj.acc.x2 =obj.acc.x2 + val;
	}
}

var update_phys_objs =function(phys_obj_array){
	for (var i = 0; i < phys_obj_array.length; i++) {
		if(!(phys_obj_array[i] instanceof Environment)){
			phys_obj_array[i].move();
			phys_obj_array[i].update();
		}
	}
}

var CollisionHandler = function(){
	this.ignore_memory = [[],[],[]]; //ignore collision env vs. env
	this.preP_memory   = [[],[],[]]; //only execute this.preProcessorCollision once -> prepP[i][j]= true;
	this.calcConsequences = function(obj_0,obj_1,intersect){
		var obj0 = obj_0.shape;
		var obj1 = obj_1.shape;
		var param0 = obj0.closestParam(intersect);
		var param1 = 0.88 //obj1.closestParam(intersect);
		var coll_tangent0 = obj0.tangent(param0);
		var coll_tangent1 = obj1.tangent(param1);
        //calc arithmetically averaged collision normal
        //convert array to vector
        var collt0 = convertArray2Vector(coll_tangent0);
        var collt1 = convertArray2Vector(coll_tangent1);
        var colltFinal = biSectorVector(collt0,collt1);
        colltFinal.x1 = 1;
        colltFinal.x2 = 0;
        colltFinal.x3 = 0;
        //normal vecto of colltFinal :
        var normal_vec = colltFinal.normal_vec();
        var angle_coll = Math.atan(colltFinal.x1/colltFinal.x2);
        if(obj_0.instance == 0){
        	if(obj_1.instance == 1){
				//calc environment vs. circle
				var origin = new Vector2d(0,0,0);
				var trans_speed1 = obj_1.speed.rotate(angle_coll,origin);

				var trans_speed_new= new Vector2d(0,0,0);

				trans_speed_new.x1 = -trans_speed1.x1*obj_1.material.e;
                trans_speed_new.x2 = trans_speed1.x2; //stays without friction
                //trans_speed_new[2] = 0; //stays zero, rotation doesnt matter without friction
                
                //convert speeds back to global directions
                obj_1.speed = trans_speed_new.inv_rotate(angle_coll,origin);
               	//correction of penetration of the physical objects due to the finite small time tick
               	//
               	var correction = Math.abs(trans_speed1.x1)*tick/4; 
               	obj_1.pos = obj_1.pos.add(normal_vec.multi_scalar(correction))

            }else if(obj_1.instance == 2){
				//calc environment vs. ellipse
					var origin = new Vector2d(0,0,0);
				var trans_speed0 = obj_1.speed.rotate(angle_coll,origin);
        		var trans_speed_new= new Vector2d(0,0,0);
        		trans_speed_new.x1 = -trans_speed0.x1*obj_1.material.e;
        		trans_speed_new.x2 = trans_speed0.x2;
        		var intersect_vec = convertArray2Vector(intersect)
        		var line = new Line2d(colltFinal,intersect_vec)
        		var massInertia = obj_1.mass*(Math.pow(obj_1.x1axis,2)+Math.pow(obj_1.x2axis,2)/5); 
        		var x_dis	    = -line.distance_point(obj_1.center)
        		obj_1.speed= trans_speed_new.inv_rotate(angle_coll,origin);
        		console.log("--------------------------------------------------------")
        		console.log("-------------------Normal Vec----------------")
        		console.log("--------------------------------------------------------")
        		console.log(colltFinal,x_dis,obj_1.speed.x3)
        		console.log(obj_0.speed.x3,'-(',trans_speed_new.x,'+',trans_speed0.x1,')*',obj_1.mass,'*',x_dis,'/',massInertia)
        			obj_1.speed.x3 = obj_1.speed.x3-(trans_speed_new.x1-trans_speed0.x1)*obj_1.mass*x_dis/massInertia;
				console.log('obj_1.speed.x3:     ',obj_1.speed.x3);
                var correction = Math.abs(trans_speed0.x1)*tick*correction_constant;
                obj_1.pos = obj_1.pos.add(normal_vec.multi_scalar(correction));
            }
        }else if(obj_0.instance == 1){
       		if(obj_1.instance == 0){
       			var origin = new Vector2d(0,0,0);
       			var trans_speed0 = obj_0.speed.rotate(angle_coll,origin);
       			var trans_speed_new= new Vector2d(0,0,0);
       			trans_speed_new.x1 = -trans_speed0.x1*obj_0.material.e;
                trans_speed_new.x2 = trans_speed0.x2; //stays without friction
                //trans_speed_new[2] = 0; //stays zero, rotation doesnt matter without friction
                //convert speeds back to global directions
                obj_0.speed = trans_speed_new.inv_rotate(angle_coll,origin);
               	//correction of penetration of the physical objects due to the finite small time tick
               	var correction = Math.abs(trans_speed0.x1)*tick/4; 
               	obj_0.pos = obj_0.pos.add(normal_vec.multi_scalar(correction))
            }else if(obj_1.instance == 2){
				//calc circle vs. ellipse
			}else if(obj_1.instance == 1){
				//calc circle vs. circle
			}
		}else if(obj_0.instance == 2){
			if(obj_1.instance == 0){
				//calc ellipse vs. enviroment
				console.log(obj_0.speed)
				var origin = new Vector2d(0,0,0);
				console.log("-----------------------------Speed pre collision---------------------------")
				console.log('obj_1.speed:     ',obj_0.speed);
				var trans_speed0 = obj_0.speed.rotate(angle_coll,origin);
				console.log("----------------------Speed pre collision transformed----------------------------------")
				console.log('trans_speed0:     ',trans_speed0);
        		var trans_speed_new= new Vector2d(0,0,0);
        		trans_speed_new.x1 = -trans_speed0.x1*obj_0.material.e;
        		trans_speed_new.x2 = trans_speed0.x2;
        		console.log("----------------------Speed after collision transformed----------------------------------")
        		console.log('trans_speed0:     ',trans_speed_new);
        		var intersect_vec = convertArray2Vector(intersect)
        		var line = new Line2d(colltFinal,intersect_vec)
        		var massInertia = obj_0.mass*(Math.pow(obj_0.x1axis,2)+Math.pow(obj_0.x2axis,2)/5); 
        		var x_dis	    = -line.distance_point(obj_0.center)
        		obj_0.speed= trans_speed_new.inv_rotate(angle_coll,origin);
        		console.log("----------------------Speed after collision ----------------------------------")
        		console.log('obj_0.speed:     ',obj_0.speed);
        		console.log("--------------------------------------------------------")
        		console.log("--------------------------------------------------------")
        		console.log("-------------------Normal Vec----------------")
        		console.log("--------------------------------------------------------")
        		console.log(colltFinal,x_dis,obj_0.speed)
        		console.log(obj_0.speed.x3,'-(',trans_speed_new.x1,'-',trans_speed0.x1,')*',obj_0.mass,'*',x_dis,'/',massInertia)
        		obj_0.speed.x3 = obj_0.speed.x3-(trans_speed_new.x1-trans_speed0.x1)*obj_0.mass*x_dis/massInertia;
				console.log('obj_1.speed.x3:     ',obj_0.speed);
                var correction = Math.abs(trans_speed0.x1)*tick*correction_constant; 
                obj_0.pos = obj_0.pos.add(normal_vec.multi_scalar(correction))
           	}else if(obj_1.instance == 1){
				//calc ellipse vs. circle
			}else if(obj_1.instance == 2){
				//calc ellipse vs. ellipse
			}
		}
	}
	this.ignoreCollision = function(i,j){
		this.ignore_memory[i][j]=true;
	}
	this.init = function (phys_obj_array){
		for (var i = 0; i < phys_obj_array.length; i++) {
			inner: for (var j = i+1; j < phys_obj_array.length; j++) {
				if(this.ignore_memory[i][j]==true ){
					continue inner;
				}else if(phys_obj_array[i].instance == 0 && phys_obj_array[j].instance == 0) {
					this.ignore_memory[i][j]==true; 
					continue inner;
				};
				var threshold =threshold_constant*Math.max(phys_obj_array[i].speed.normAll(),phys_obj_array[j].speed.normAll());
				var intersect = verb.geom.Intersect.curves( phys_obj_array[i].shape, phys_obj_array[j].shape, 1e-16 );
				if(intersect==0) {
					continue inner;
				}else if(time_overall-phys_obj_array[i].lastcollision >threshold*tick){ //numerical trick to control last collision maybe as array between objects
					console.log('Collision Überwachung--------------------------------------------------------------------')
					console.log('Collision Überwachung: objekt1 objekt2 ,zeit, Zeit seit letztem Zusammenstoß', phys_obj_array[i].addcount,phys_obj_array[j].addcount,time_overall,phys_obj_array[i].lastcollision)
					phys_obj_array[i].lastcollision= time_overall;
					if(intersect.length>=2){
						var intersect_average = [(intersect[0].point0[0]+intersect[1].point0[0])/2, (intersect[0].point0[1]+intersect[1].point0[1])/2,0]
						this.calcConsequences(phys_obj_array[i],phys_obj_array[j],intersect_average)
           			//pass objs and intersect to this.calcConsequences
           			}else{
           				var intersect_single = [intersect[0].point0[0], intersect[0].point0[1],0];
           				this.calcConsequences(phys_obj_array[i],phys_obj_array[j],intersect_single)
           			}
       			}
   			}
   		}
   	}			
}

var Phys_obj = function(pos,mass,speed,acc,center){
	this.pos = pos;  //pos.x1 =x position, pos.x2 =y position, pos.x3 =orientation_angle,
	this.mass = mass;
	this.speed = speed || new Vector2d(0,0,0); //speed.x1= speed x-direction... speed.x3 = rotation x3 axis
	this.acc = acc || new Vector2d(0,0,0); // acc = [0,0,0]
	this.center = center || new Vector2d(0,0,0);
	this.instance = undefined;
	this.lastcollision = 0;
	this.shapeData = {
		degree: 0,
		knotvec: 0,
		controlpoints: 0,
		weights: 0
	}; //every shape is generated with NURBS basic functions //shapeData: [knotvec, controlpoints,weights]
	this.material = new Material(0,0);
	this.generateNurbsData = function() { //convert shapeData to verb nurbsData
		this.nurbsData = new verb.core.NurbsCurveData(this.shapeData.degree,this.shapeData.knotvec.slice(),verb.core.Eval.homogenize1d(this.shapeData.controlpoints,this.shapeData.weights),true);
	}
	this.generateShape= function() { //generate shape from verbData
		this.shape = new verb.geom.NurbsCurve(this.nurbsData);
	}
	this.move = function() { //update pos, speed //x3 values are rotationvalues
		this.pos = this.pos.addAll((this.speed.multi_scalarAll(tick)).addAll(this.acc.multi_scalarAll(0.5*Math.pow(tick,2))));//this.shape = new verb.geom.NurbsCurve(this.nurbsData);
		this.speed = this.speed.addAll(this.acc.multi_scalarAll(tick));
	}
	this._draw = function() { //draw to scene
		if(this.instance !=  0 ){
			scene.remove(scene.children[this.addcount+4]);
		}
			addCurveToScene(this.shape.toThreeGeometry());
	}
}

var Circle = function(radius,pos,mass,material,addcount){
	this.radius = checkandYield(radius,"radius");
	this.pos 	= pos;
	this.addcount= addcount;
	this.center = new Vector2d(this.pos.x1+this.radius,this.pos.x2+this.radius,0);
	this.mass   = checkandYield(mass,"mass");
	this.shapeData.degree = 2;
	this.shapeData.knotvec = knotvec_circ;
	this.shapeData.weights = ptsweights_circ;
	this.material = checkandYield(material,"Material");
	this.instance = 1; //1 equals circle
	this.generateCpts = function(){
		var p1b= [this.pos.x1,                this.pos.x2+this.radius,     0];
		var p2b= [this.pos.x1,                this.pos.x2,                 0]; 
		var p3b= [this.pos.x1+this.radius,    this.pos.x2,                 0]; 
		var p4b= [this.pos.x1+2*this.radius,  this.pos.x2,                 0]; 
		var p5b= [this.pos.x1+2*this.radius,  this.pos.x2+this.radius,     0]; 
		var p6b= [this.pos.x1+2*this.radius,  this.pos.x2+2*this.radius,   0]; 
		var p7b= [this.pos.x1+this.radius,    this.pos.x2+2*this.radius,   0];
		var p8b= [this.pos.x1,                this.pos.x2+2*this.radius,   0];
		var ptsb= [p1b,p2b, p3b,p4b, p5b,p6b,p7b,p8b,p1b];
		return ptsb;
	}
	this.shapeData.controlpoints = this.generateCpts();
	this.generateNurbsData();
	this.generateShape();
	this.update = function(){
		this.shapeData.controlpoints = this.generateCpts();
		this.generateNurbsData();
		this.generateShape();
		this._draw();
		this.center = new Vector2d(this.pos.x1+this.radius,this.pos.x2+this.radius,0);
	}
}

Circle.prototype = new Phys_obj;
Circle.prototype.constructor = Circle;

var Environment = function(degree,controlpoints,knotvec,weights,addcount){
	this.shapeData = {
		degree: degree,
		knotvec: knotvec,
		controlpoints: controlpoints,
		weights: weights
	};
	this.addcount= addcount;
	this.instance = 0 ; //0 equals env
	this.speed = new Vector2d(0,0,0)
	this.generateNurbsData = function() { //convert shapeData to verb nurbsData
		this.nurbsData = new verb.core.NurbsCurveData(this.shapeData.degree,this.shapeData.knotvec.slice(),verb.core.Eval.homogenize1d(this.shapeData.controlpoints,this.shapeData.weights),false);
	}
	this.generateShape= function() { //generate shape from verbData
		this.shape = new verb.geom.NurbsCurve(this.nurbsData);
	}
	this.generateNurbsData();
	this.generateShape();
	this._draw();
}

Environment.prototype = new Phys_obj;
Environment.prototype.constructor = Environment;

var Ellipse = function(x1axis,x2axis,pos,mass,material,addcount){
	this.x1axis = checkandYield(x1axis,"x1axis");
	this.x2axis = checkandYield(x2axis,"x2axis");
	this.pos 	= checkandYield(pos,"pos");
	this.mass   = checkandYield(mass,"mass");
	this.center = new Vector2d(this.pos.x1+this.x1axis,this.pos.x2+this.x2axis,0);
	this.massIntertia = 2;//this.mass*()
	this.shapeData.degree = 2;
	this.addcount= addcount;
	this.shapeData.knotvec = knotvec_circ;
	this.shapeData.weights = ptsweights_circ;
	this.material = checkandYield(material,"Material");
	this.instance = 2 ; //2 equals ellipse
	this.generateCpts = function(){
		var p1b= new Vector2d(this.pos.x1,                this.pos.x2+this.x2axis,     0);
		var p2b= new Vector2d(this.pos.x1,                this.pos.x2,                 0); 
		var p3b= new Vector2d(this.pos.x1+this.x1axis,    this.pos.x2,                 0); 
		var p4b= new Vector2d(this.pos.x1+2*this.x1axis,  this.pos.x2,                 0); 
		var p5b= new Vector2d(this.pos.x1+2*this.x1axis,  this.pos.x2+this.x2axis,     0); 
		var p6b= new Vector2d(this.pos.x1+2*this.x1axis,  this.pos.x2+2*this.x2axis,   0); 
		var p7b= new Vector2d(this.pos.x1+this.x1axis,    this.pos.x2+2*this.x2axis,   0);
		var p8b= new Vector2d(this.pos.x1,                this.pos.x2+2*this.x2axis,   0);
		var ptsb= [p1b,p2b, p3b,p4b, p5b,p6b,p7b,p8b,p1b];
		for (var i = 0; i < ptsb.length; i++) {
			ptsb[i] = convertVector2Array((ptsb[i]).rotate(this.pos.x3,this.center));
		};
		return ptsb;
	}
	this.shapeData.controlpoints = this.generateCpts();
	this.generateNurbsData();
	this.generateShape();
	this.update = function(){
		this.shapeData.controlpoints = this.generateCpts();
		this.generateNurbsData();
		this.generateShape();
		this._draw();
		this.center = new Vector2d(this.pos.x1+this.x1axis,this.pos.x2+this.x2axis,0);
	}
}

Ellipse.prototype = new Phys_obj;
Ellipse.prototype.constructor = Ellipse;