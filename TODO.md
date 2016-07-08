NOTES
-------------------------

**Naming conventions**
- *odometer*: the distance the car has covered so far (along it's path).
- *carLocation [x,y,theta]*: The absolute location of the car center point in 2D space, calculated from the beginning of the simulation. Theta is the heading angle, 0 being car is pointing "upwards".
- *roadLocation [x,y,theta]*: The absolute location of a road point (on the road centerline) in 2D space, calculated from the beginning of the simulation. Theta is the heading angle, 0 being car is pointing "upwards".
- *travel*: The distance the car has covered along the road, expressed with the road's curve parameter. The car's travel.
- *carPosition / carPositionOnRoad*: car position relative to a specific roadParam. "position" is used mainly when car position is given relative to the road.
- *roadParam*: The curveParameter of the road, meaning a point on the road centerline, and also how much ditance the road has covered along it's path. It's unit is meters.
- *roadVisibility*: How much road is visible ahead car, expressed with *roadParam*.
- *normalPosition*: the position on the road, perpendicular to the centerline, in the range of [-1,1]
- *crossPosition*: the position on the road, perpendicular to the centerline, in meters, signed
- radii and angles are negative when bend/turn is to the left (heading as well!! watch out when using rotation transforms!)

```
Trajectory stuff is incomplete all over the place, probably never will be finished
	Section classes should be finished
	trajectory planning in CarDriver should be continued

Steering control based turning approach
	obstacle avoidance
		calculate point beside obstacle where to pass
		I should be able to calculate the distance of the avoidance manouver (start to steer with max allowed radius by friction and max allowed by car)
		if avoidDist < obstDist, calculate an avoidance with all the available distance (how? steering control P?...no)
		if avoidDist > obstDist, start to slow down with max acc (and calculate again)
		set target normal pos for steering control, and done
	bend turn
		lane control:
			car calculates current turn radius -> get shift in given odo
			roadRadius at a given point is known. in bend roadRadius-carTurnRadius=dR gives the rate of shift. rate of shift with simInterval->delta Odo gives delta crossPos
```

TODO
--------------------------

**CarDriver**

- car deviates away from road: possible road <> car coordinate deviation? Cumulative error? sg. with wheelAngle (at Car::simUpdate()?)
- roadWidth variation
  - only in straights from start to end (next segment continues with end width!)
- signal something when maxTangentialAccel<epsilon? possibleCrash or sg?
- what if I'm on a turn, limited by centripAcc to brake for a coming, second turn, which comes after a straight segment on which I can brake fully? Somehow I should calculate decelDistance based on the road ahead... (not so hard..)
- if cruise speed is updated from gui, react to it! now only updates on new straight segment start
- decelDistance should consider currentSpeed*dT, where dT is simulation interval. can lead to tractionLoss/crash.
- when braking for a bend, and braking doesn't end until the start of the bend, car will start to steer, still with the max tangent accel, calculated without turn -> quaranteed traction loss.
- USE CARHEADINGoNROAD TO FOLLOW ROAD (setting wheelAngle at the direction of the road will pull the car always in the right direction!)
- obstacle targetAvoidanceCrossPos is always set in every iter until obstFeaturePoint is left behind..


**Simulation**

  - catch unavoidable and final tractionLoss and crash signals from driver, relay to simView?

**Visualization, GUI**

  - separate slider for tangential accel (now +tangential and +normal are the same)
  
**Other**

