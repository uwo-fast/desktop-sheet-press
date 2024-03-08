wall_thick=4;

breaker_arc_radius=7.5;
breaker_arc_arc_sep=9.3;
breaker_arc_width=4;
breaker_tab_height=5.25;
breaker_tab_width=3;

breaker_arc_arc_sep_tabs=breaker_arc_arc_sep+breaker_tab_width;

z_fite=0.05;

breaker_tab_ratio=0.75;
breaker_tab_ratioi=1-breaker_tab_ratio;

recep_thick=wall_thick*1.5;

breaker_height = 2*breaker_arc_radius*cos(asin((breaker_arc_radius-breaker_arc_width)/breaker_arc_radius));

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
  cube([breaker_arc_arc_sep,breaker_height,recep_thick],center=true);
   
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