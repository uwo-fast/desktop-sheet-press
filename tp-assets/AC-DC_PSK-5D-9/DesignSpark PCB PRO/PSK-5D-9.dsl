SamacSys ECAD Model
13841802/916955/2.50/4/2/Power Supply

DESIGNSPARK_INTERMEDIATE_ASCII

(asciiHeader
	(fileUnits MM)
)
(library Library_1
	(padStyleDef "c165_h110"
		(holeDiam 1.1)
		(padShape (layerNumRef 1) (padShapeType Ellipse)  (shapeWidth 1.650) (shapeHeight 1.650))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 1.650) (shapeHeight 1.650))
	)
	(padStyleDef "s165_h110"
		(holeDiam 1.1)
		(padShape (layerNumRef 1) (padShapeType Rect)  (shapeWidth 1.650) (shapeHeight 1.650))
		(padShape (layerNumRef 16) (padShapeType Rect)  (shapeWidth 1.650) (shapeHeight 1.650))
	)
	(textStyleDef "Default"
		(font
			(fontType Stroke)
			(fontFace "Helvetica")
			(fontHeight 50 mils)
			(strokeWidth 5 mils)
		)
	)
	(patternDef "PSK5D9" (originalName "PSK5D9")
		(multiLayer
			(pad (padNum 1) (padStyleRef s165_h110) (pt 0.000, 0.000) (rotation 90))
			(pad (padNum 2) (padStyleRef c165_h110) (pt 0.000, -5.080) (rotation 90))
			(pad (padNum 3) (padStyleRef c165_h110) (pt 20.320, -10.160) (rotation 90))
			(pad (padNum 4) (padStyleRef c165_h110) (pt 20.320, 0.000) (rotation 90))
		)
		(layerContents (layerNumRef 18)
			(attr "RefDes" "RefDes" (pt 10.160, -10.160) (textStyleRef "Default") (isVisible True))
		)
		(layerContents (layerNumRef 28)
			(line (pt -2.54 2.54) (pt 22.86 2.54) (width 0.1))
		)
		(layerContents (layerNumRef 28)
			(line (pt 22.86 2.54) (pt 22.86 -22.86) (width 0.1))
		)
		(layerContents (layerNumRef 28)
			(line (pt 22.86 -22.86) (pt -2.54 -22.86) (width 0.1))
		)
		(layerContents (layerNumRef 28)
			(line (pt -2.54 -22.86) (pt -2.54 2.54) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -2.54 2.54) (pt 22.86 2.54) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt 22.86 2.54) (pt 22.86 -22.86) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt 22.86 -22.86) (pt -2.54 -22.86) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt -2.54 -22.86) (pt -2.54 2.54) (width 0.2))
		)
		(layerContents (layerNumRef 30)
			(line (pt -3.54 3.54) (pt 23.86 3.54) (width 0.1))
		)
		(layerContents (layerNumRef 30)
			(line (pt 23.86 3.54) (pt 23.86 -23.86) (width 0.1))
		)
		(layerContents (layerNumRef 30)
			(line (pt 23.86 -23.86) (pt -3.54 -23.86) (width 0.1))
		)
		(layerContents (layerNumRef 30)
			(line (pt -3.54 -23.86) (pt -3.54 3.54) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -3.2 0) (pt -3.2 0) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(arc (pt -3.15, 0) (radius 0.05) (startAngle 180.0) (sweepAngle 180.0) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -3.1 0) (pt -3.1 0) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(arc (pt -3.15, 0) (radius 0.05) (startAngle .0) (sweepAngle 180.0) (width 0.1))
		)
	)
	(symbolDef "PSK-5D-9" (originalName "PSK-5D-9")

		(pin (pinNum 1) (pt 0 mils 0 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -25 mils) (rotation 0]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 2) (pt 0 mils -100 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -125 mils) (rotation 0]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 3) (pt 1100 mils -100 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 870 mils -125 mils) (rotation 0]) (justify "Right") (textStyleRef "Default"))
		))
		(pin (pinNum 4) (pt 1100 mils 0 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 870 mils -25 mils) (rotation 0]) (justify "Right") (textStyleRef "Default"))
		))
		(line (pt 200 mils 100 mils) (pt 900 mils 100 mils) (width 6 mils))
		(line (pt 900 mils 100 mils) (pt 900 mils -200 mils) (width 6 mils))
		(line (pt 900 mils -200 mils) (pt 200 mils -200 mils) (width 6 mils))
		(line (pt 200 mils -200 mils) (pt 200 mils 100 mils) (width 6 mils))
		(attr "RefDes" "RefDes" (pt 950 mils 300 mils) (justify Left) (isVisible True) (textStyleRef "Default"))

	)
	(compDef "PSK-5D-9" (originalName "PSK-5D-9") (compHeader (numPins 4) (numParts 1) (refDesPrefix PS)
		)
		(compPin "1" (pinName "AC(N)") (partNum 1) (symPinNum 1) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "2" (pinName "AC(L)") (partNum 1) (symPinNum 2) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "4" (pinName "-VO") (partNum 1) (symPinNum 3) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "5" (pinName "+VO") (partNum 1) (symPinNum 4) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(attachedSymbol (partNum 1) (altType Normal) (symbolName "PSK-5D-9"))
		(attachedPattern (patternNum 1) (patternName "PSK5D9")
			(numPads 4)
			(padPinMap
				(padNum 1) (compPinRef "1")
				(padNum 2) (compPinRef "2")
				(padNum 3) (compPinRef "4")
				(padNum 4) (compPinRef "5")
			)
		)
		(attr "Mouser Part Number" "490-PSK-5D-9")
		(attr "Mouser Price/Stock" "https://www.mouser.co.uk/ProductDetail/CUI-Inc/PSK-5D-9?qs=DRkmTr78QAS7xBfnsfADZA%3D%3D")
		(attr "Manufacturer_Name" "CUI Inc.")
		(attr "Manufacturer_Part_Number" "PSK-5D-9")
		(attr "Description" "AC/DC Power Modules ac-dc, 5 W, 9 Vdc, single output, encapsulated PCB, 85~305 Vac")
		(attr "Datasheet Link" "")
		(attr "Height" "18.1 mm")
	)

)