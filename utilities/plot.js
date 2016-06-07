 function timestamp() {
        return window.performance && window.performance.now ? window.performance.now() : new Date().getTime();
    };
 
    function frame() {
        
       // tick    = 1/10;
        

        threeRender();
        };
var now =0;
var last =0;
//tick=1/100000;
    function threeRender(){
        var controls = new THREE.OrbitControls(camera, renderer.domElement);
        function render() {
            now  = timestamp();

            

            init(); 
           // update();
            //render_ball();

            //ball1._remove(ball1._create_draw_obj());
            
            requestAnimationFrame( render );
            renderer.render( scene, camera );
            last  = timestamp();
        }
    render();
    }