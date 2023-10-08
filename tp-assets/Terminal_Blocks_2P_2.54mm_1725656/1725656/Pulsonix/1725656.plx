PULSONIX_LIBRARY_ASCII "SamacSys ECAD Model"
//246260/916955/2.50/2/2/Connector

(asciiHeader
	(fileUnits MM)
)
(library Library_1
	(padStyleDef "c55_h110"
		(holeDiam 1.1)
		(padShape (layerNumRef 1) (padShapeType Ellipse)  (shapeWidth 0.550) (shapeHeight 0.550))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 0.550) (shapeHeight 0.550))
	)
	(padStyleDef "s165_h110"
		(holeDiam 1.1)
		(padShape (layerNumRef 1) (padShapeType Rect)  (shapeWidth 1.650) (shapeHeight 1.650))
		(padShape (layerNumRef 16) (padShapeType Rect)  (shapeWidth 1.650) (shapeHeight 1.650))
	)
	(padStyleDef "c165_h110"
		(holeDiam 1.1)
		(padShape (layerNumRef 1) (padShapeType Ellipse)  (shapeWidth 1.650) (shapeHeight 1.650))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 1.650) (shapeHeight 1.650))
	)
	(textStyleDef "Normal"
		(font
			(fontType Stroke)
			(fontFace "Helvetica")
			(fontHeight 1.27)
			(strokeWidth 0.127)
		)
	)
	(patternDef "1725656" (originalName "1725656")
		(multiLayer
			(pad (padNum 1) (padStyleRef s165_h110) (pt 0.000, 0.000) (rotation 90))
			(pad (padNum 2) (padStyleRef c165_h110) (pt 2.540, 0.000) (rotation 90))
			(pad (padNum 3) (padStyleRef c55_h110) (pt 0.000, -2.540) (rotation 90))
			(pad (padNum 4) (padStyleRef c55_h110) (pt 2.540, -2.540) (rotation 90))
		)
		(layerContents (layerNumRef 18)
			(attr "RefDes" "RefDes" (pt 1.270, 0.000) (textStyleRef "Normal") (isVisible True))
		)
		(layerContents (layerNumRef 28)
			(line (pt -1.5 -3.1) (pt 4.04 -3.1) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt 4.04 -3.1) (pt 4.04 3.1) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt 4.04 3.1) (pt -1.5 3.1) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt -1.5 3.1) (pt -1.5 -3.1) (width 0.025))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -2.5 4.1) (pt 5.04 4.1) (width 0.1))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 5.04 4.1) (pt 5.04 -4.1) (width 0.1))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 5.04 -4.1) (pt -2.5 -4.1) (width 0.1))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -2.5 -4.1) (pt -2.5 4.1) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -0.9 -3.1) (pt -1.5 -3.1) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt -1.5 -3.1) (pt -1.5 3.1) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt -1.5 3.1) (pt 4.04 3.1) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt 4.04 3.1) (pt 4.04 -3.1) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt 4.04 -3.1) (pt 3.3 -3.1) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt -2.1 0) (pt -2.1 0) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(arc (pt -2.05, 0) (radius 0.05) (startAngle 180.0) (sweepAngle 180.0) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt -2 0) (pt -2 0) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(arc (pt -2.05, 0) (radius 0.05) (startAngle .0) (sweepAngle 180.0) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt -2.1 0) (pt -2.1 0) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(arc (pt -2.05, 0) (radius 0.05) (startAngle 180.0) (sweepAngle 180.0) (width 0.2))
		)
	)
	(symbolDef "1725656" (originalName "1725656")

		(pin (pinNum 1) (pt 0 mils 0 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -25 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 2) (pt 0 mils -100 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -125 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(line (pt 200 mils 100 mils) (pt 600 mils 100 mils) (width 6 mils))
		(line (pt 600 mils 100 mils) (pt 600 mils -200 mils) (width 6 mils))
		(line (pt 600 mils -200 mils) (pt 200 mils -200 mils) (width 6 mils))
		(line (pt 200 mils -200 mils) (pt 200 mils 100 mils) (width 6 mils))
		(attr "RefDes" "RefDes" (pt 650 mils 300 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))
		(attr "Type" "Type" (pt 650 mils 200 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))

	)
	(compDef "1725656" (originalName "1725656") (compHeader (numPins 2) (numParts 1) (refDesPrefix J)
		)
		(compPin "1" (pinName "1") (partNum 1) (symPinNum 1) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "2" (pinName "2") (partNum 1) (symPinNum 2) (gateEq 0) (pinEq 0) (pinType Unknown))
		(attachedSymbol (partNum 1) (altType Normal) (symbolName "1725656"))
		(attachedPattern (patternNum 1) (patternName "1725656")
			(numPads 2)
			(padPinMap
				(padNum 1) (compPinRef "1")
				(padNum 2) (compPinRef "2")
			)
		)
		(attr "Mouser Part Number" "651-1725656")
		(attr "Mouser Price/Stock" "https://www.mouser.co.uk/ProductDetail/Phoenix-Contact/1725656?qs=Ul7CXFMnlWWQeccayYbRmw%3D%3D")
		(attr "Manufacturer_Name" "Phoenix Contact")
		(attr "Manufacturer_Part_Number" "1725656")
		(attr "Description" "PCB terminal block, nominal current: 6 A, rated voltage (III/2): 160 V, nominal cross section: 0.5 mm?, Number of potentials: 2, Number of rows: 1, Number of positions per row: 2, product range: MPT 0,5, pitch: 2.54 mm, connection method: Screw connection with tension sleeve, mounting: Wave soldering, conductor/PCB connection direction: 0 ?, color: green, Pin layout: Linear pinning, Solder pin [P]: 3.5 mm, type of packaging: packed in cardboard")
		(attr "<Hyperlink>" "https://www.phoenixcontact.com/online/portal/us/?uri=pxc-oc-itemdetail:pid=1725656&library=usen&pcck=P-11-01-05&tab=1&selectedCategory=ALL")
		(attr "<Component Height>" "8.65")
		(attr "<STEP Filename>" "1725656.stp")
		(attr "<STEP Offsets>" "X=4.04;Y=-3.1;Z=0.5")
		(attr "<STEP Rotation>" "X=90;Y=0;Z=90")
	)

)