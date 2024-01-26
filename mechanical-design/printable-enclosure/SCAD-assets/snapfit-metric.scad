// Adapted by Cameron K. Brooks on 2024-01-26 from the following source: 
// https://www.thingiverse.com/thing:1799553/files


dia = 2;	// Snap fit diameter
lip = 0.75;	// Snap fit lip (adds to diameter)
col_h = dia * 2;
//col_h = 5;    // Snap fit Mini column height

slot_ratio = 0.7;  // Slot ratio

col_tol = 0.1;  // Column Tolerance
neg_tol = 0.5;  // Tolerance for the Neagtive Snapfit

$fn = 100;


module snapfit(diameter = dia, lip = lip, column_height = col_h, column_tolerance = col_tol, negative_tolerance = neg_tol, slot_ratio = slot_ratio) {
	z_fite = $preview ? 0.05 : 0.0;  // Z offset for preview
	// Calculated Snapfit Parameters
	head_h = (diameter+lip)/2;    // Snap fit height
	head_r1 = (diameter+lip)/2;    // Snap fit radius 1
	head_r2 = calculateRadFromAngle(180-60, head_r1, head_h);    // Snap fit radius 2
	col_r = diameter/2;    // Mini column radius

    difference() {
        union() {
            translate([0, 0, col_h+head_h/2])
                cylinder(r1 = head_r1, r2 = head_r2, h = head_h, center = true); // snapfit head

            translate([0, 0, col_h/2])
                cylinder(r = col_r-col_tol, h = col_h, center = true); // Mini column
        }
        translate([0, 0, (col_h+head_h+z_fite)-(slot_ratio*(col_h+head_h))/2])
            cube([head_r1/3, head_r1*2, slot_ratio*(col_h+head_h)], center = true); // Cut in snap fit
    }
}

module snapfit_neg() {
    union() {
        translate([0, 0, col_h])
            cylinder(r1 = head_r1 + neg_tol, r2 = head_r2 + neg_tol, h = head_h + neg_tol, center = true); // snapfit head
        translate([0, 0, col_h/2])
            cylinder(r = head_r1 - neg_tol/2, h = col_h + neg_tol, center = true); // Mini column
    }
}

// Function to calculate radius based on angle, initial radius, and height
function calculateRadFromAngle(angle, r, height) = 
    let(norm_angle = angle % 90)
    (angle < 90)
        ? r + tan((abs(90 - norm_angle))) * height
        : r - tan((norm_angle)) * height;

// Example Parameters
w = 20;   // Column diameter
h = 10;   // Main column height

module example() {
	difference() {
		union() {
			cylinder(r = w/2, h = h, center = true); // Main cylinder
			translate([0, 0, h/2])
			snapfit();
		}
		translate([0, 0, -h/2]) 
		snapfit_neg();
	}
}

snapfit();