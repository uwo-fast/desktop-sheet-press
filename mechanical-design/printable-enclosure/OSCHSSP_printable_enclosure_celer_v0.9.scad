include <C:\Program Files\OpenSCAD\libraries\MarksEnclosureHelper\hingebox_code.scad> 

hingedbox( box_def );
//myperf( box_def );

// show ghosts, top/bottom and magnets
VIS=false;  

// extra space between top and bottom
PART_SPACE=10; 

box_height=160;
box_width=160;
box_depth=80;

box_def = [160, 160, 80]; 
wall_thick = 4;  

top_rat = 0.20;  
lip_rat= 0.10;  

hinge_points = [1/3, 2/3];
hinge_len = 24; 

// where to put catch clasps
catch_points=[0.5];

// width of catch clasp (<1 * box_def.x, >1 actual units)
catch_wide = 0.20;

// width of catch clasp bottom (<1 * catch_wide, >1 actual units)
catch_wide_bottom = 0.8;

// width of tooth /  cutout in catch (* catch_wide_bottom)
catch_tooth_xrat = 0.7;

// where on the bottom height the tooth rides
catch_tooth_zrat = 0.7;

// thickness of thicker catch bars; and tooth
catch_thick = wall_thick * 1.5; //wall_thick * 2;

// thickness of catch inner fill
catch_inner_thick = false; //wall_thick;

// $fn for catch corner sphere shapes
catch_fn = 32;

// extra height to take from cutout
catch_hole_xtra = false; //CLEAR;

// space between tooth and catch when closed
catch_offset = false; //CLEAR/2;

// thickness of catch foot piece
catch_foot_thick = catch_thick *1.25;//catch_thick *2;

// width of foot bar grip piece (* catch_wide_bottom)
catch_foot_xrat = 1.0;

screw_points = [0.4, 0.6];
screw_punchbottom = true;


module myperf(d) {
    cyl_grid(d, 0.60, 3, 4.8, 6 );
} 

module cutout_top( d ) { myperf(d); }
module cutout_left( d ) { myperf(d); }
module cutout_right( d ) { myperf(d); }

////BOTTOM CUT OUT

//vars for recep
tab_ratio=0.75;
tab_ratioi=1-tab_ratio;
tab_multi=3;

recep_width = 27.3;
recep_notch_width=6.5;
recep_notch_height=1.5;
recep_height=39.3;
hole_sep=36;
hole_dia=3;
z_fite=0.05;

recep_thick=wall_thick*1.5;

//vars for breaker

breaker_arc_radius=7.5;
breaker_arc_arc_sep=9.3;
breaker_arc_width=4;
breaker_tab_height=5.25;
breaker_tab_width=3;

breaker_arc_arc_sep_tabs=breaker_arc_arc_sep+breaker_tab_width;

breaker_tab_ratio=0.5;
breaker_tab_ratioi=1-breaker_tab_ratio;

breaker_height = 2*breaker_arc_radius*cos(asin((breaker_arc_radius-breaker_arc_width)/breaker_arc_radius));

// bottom cutout
module cutout_bottom( d ) { 
  //recep
  translate([box_height-55,box_width-35,recep_thick/4])
  rotate([0,0,90])
  union() {
    difference() {
      cube([recep_width, recep_height,recep_thick*tab_ratio],center=true);
      union() {
        translate([0,recep_height/2-recep_notch_height/2,0]) cube([recep_notch_width, recep_notch_height+z_fite, recep_thick*tab_ratio+z_fite], center=true);

        translate([0,-recep_height/2+recep_notch_height/2,0]) cube([recep_notch_width, recep_notch_height+z_fite, recep_thick*tab_ratio+z_fite], center=true);
      }
    }

    translate([0,0,(recep_thick*tab_ratio)/2]) 
    difference() {
      cube([recep_width, recep_height+tab_multi*recep_notch_height*2,recep_thick*tab_ratioi],center=true);
      union() {
        translate([0,recep_height/2-recep_notch_height/2+recep_notch_height*tab_multi/2,0]) cube([recep_notch_width, recep_notch_height+tab_multi*recep_notch_height+z_fite, recep_thick*tab_ratioi+z_fite], center=true);

        translate([0,-recep_height/2+recep_notch_height/2-recep_notch_height*tab_multi/2,0]) cube([recep_notch_width, recep_notch_height+tab_multi*recep_notch_height+z_fite, recep_thick*tab_ratioi+z_fite], center=true);
      }
    }
  }
  
  //breaker
  translate([box_height-20,box_width-35,-recep_thick/4])
  rotate([0,0,90])
  union() {
    translate([-(breaker_arc_arc_sep-2*(breaker_arc_radius-breaker_arc_width))/2,0,0])
    union() {
      difference() {
        cylinder(recep_thick, breaker_arc_radius, breaker_arc_radius);

        translate([-(breaker_arc_radius-breaker_arc_width),-breaker_arc_radius,-breaker_arc_radius]) cube(breaker_arc_radius*2);
        }

        translate([breaker_arc_arc_sep-2*(breaker_arc_radius-breaker_arc_width),0,0])
      difference() {
        cylinder(recep_thick, breaker_arc_radius, breaker_arc_radius);

        translate([-breaker_arc_radius*2+(breaker_arc_radius-breaker_arc_width),-breaker_arc_radius,-breaker_arc_radius]) cube(breaker_arc_radius*2);
      }
    }

    translate([0,0,recep_thick/2])
    cube([breaker_arc_arc_sep+z_fite,breaker_height,recep_thick],center=true);
     
    translate([0,0,recep_thick*breaker_tab_ratio])
    difference() {
      
      translate([-(breaker_arc_arc_sep_tabs-2*(breaker_arc_radius-breaker_arc_width))/2,0,0]) union() {
        difference() {
          cylinder(recep_thick*breaker_tab_ratioi, breaker_arc_radius, breaker_arc_radius);

          translate([-(breaker_arc_radius-breaker_arc_width),-breaker_arc_radius,-breaker_arc_radius]) cube(breaker_arc_radius*2);
          }

          translate([breaker_arc_arc_sep_tabs-2*(breaker_arc_radius-breaker_arc_width),0,0])
        difference() {
          cylinder(recep_thick*breaker_tab_ratioi, breaker_arc_radius, breaker_arc_radius);

          translate([-breaker_arc_radius*2+(breaker_arc_radius-breaker_arc_width),-breaker_arc_radius,-breaker_arc_radius]) cube(breaker_arc_radius*2);
        }
      }
      
      union() {
        translate([-breaker_arc_radius-breaker_arc_arc_sep_tabs/2,breaker_tab_height/2,-z_fite]) cube([breaker_arc_radius*2+breaker_arc_arc_sep_tabs,breaker_arc_radius*2,recep_thick*2]);
        
          translate([-breaker_arc_radius-breaker_arc_arc_sep_tabs/2,-breaker_tab_height/2-breaker_arc_radius*2,-z_fite]) cube([breaker_arc_radius*2+breaker_arc_arc_sep_tabs,breaker_arc_radius*2,recep_thick*2]);
      }
    }
  }
  
}



//translate([(160-145)/2, (160-140)/2, 3]) cube([145,140,3]);