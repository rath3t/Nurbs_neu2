
var checkandYield = function(param,name){
	if(param || param === 0){
		return param;
	}else{
		throw new Error('Parameter not defined: '+ name);
	}
}
var convertArray2Vector = function(_array){
	return new Vector2d(_array[0],_array[1],_array[2])
}
var convertVector2Array = function(_vector){
	return [_vector.x1,_vector.x2,_vector.x3]
}
var biSectorVector =function (_v1,_v2){
	return new Vector2d((_v1.x1+_v2.x1)/2, (_v1.x2+_v2.x2)/2,0)
}