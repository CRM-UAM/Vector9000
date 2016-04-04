// CRMaze robot
// By Víctor Uceda & Carlos García
//   Club de Robótica-Mecatrónica,
//   Universidad Autónoma de Madrid
//   http://crm-uam.github.io
//
// Derived from:
//   Pololu motor STL by Andrew Taylor ( https://grabcad.com/library/pololu-micro-motor-1-250-reduction-1 )
//   Sharp IR sensor STL by Mojtaba ( https://grabcad.com/library/ir-sensor )


// Increase the resolution of default shapes
$fa = 5; // Minimum angle for fragments [degrees]
$fs = 0.5; // Minimum fragment size [mm]


largo=75;
ancho=70;
grosor=1.6;

tolerancia=0.225;

module rueda(ra=10,ancho=17){
    translate([-6,-5,1]) import("libs/motor.stl");

    color("gray") union() {
        translate([0,0,19])
            difference() {
                hull() {
                    cylinder(r=ra,h=ancho-2);
                    translate([0,0,ancho-2.01]) cylinder(r=ra-1,h=2);
                }
                difference() {
                    cylinder(r=3.3/2+tolerancia,h=ancho+1);
                    translate([-10,1.2+tolerancia,0]) cube([20,20,ancho+2]);
                }
                translate([0,0,ancho+1]) sphere(r=5/2);
                translate([0,0,-0.1]) cylinder(r=8.5,h=9);
            }
    }
   
}

rueda();

