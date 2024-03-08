
wall_thick = 4;  

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



