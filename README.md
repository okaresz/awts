## AW trial simulation

### Frameworks, dependencies

- Qt 5.6

**Build**

1. `qmake awts.pro`
2. `make`


### Naming conventions

- *odometer*: the distance the car has covered so far (along it's path).
- *carLocation [x,y,theta]*: The absolute location of the car center point in 2D space, calculated from the beginning of the simulation. Theta is the heading angle, 0 being car is pointing "upwards", turning clockwise is negative.
- *roadLocation [x,y,theta]*: The absolute location of a road point (on the road centerline) in 2D space, calculated from the beginning of the simulation. Theta is the heading angle, 0 being car is pointing "upwards", turning clockwise is negative.
- *travel*: The distance the car has covered along the road, expressed with the road's curve parameter. The car's travel.
- *carPosition / carPositionOnRoad*: car position relative to a specific roadParam. "position" is used mainly when car position is given relative to the road.
- *roadParam*: The curveParameter of the road, meaning a point on the road centerline, and also how much ditance the road has covered along it's path. It's unit is meters.
- *roadVisibility*: How much road is visible ahead car, expressed with *roadParam*.
- *normalPosition*: the position on the road, perpendicular to the centerline, in the range of [-1,1]
- *crossPosition*: the position on the road, perpendicular to the centerline, in meters, signed
- *tangential* (mainly acceleration): meaning tangent to the road centerline, or pointing to the direction of the car's movement
- *centripetal acceleration*: the centripetal acceleration of the car on a bend


### Notes

**Global**

- important settings and default parameters are saved in a configuration file. For it's location, see [Qt documentation](http://doc.qt.io/qt-5.6/qsettings.html#platform-specific-notes).
- `Simulator` is responsible to give a time interval to the simulation, calling *simUpdate()* functions of other simulation classes (like `RoadGen`, `Car`, ...)
- `Simulator` calculates the relative position and heading of the car on the road, from *carLocation* and *roadLocation*; `CarDriver` gets only this relative car position
- radii and angles are negative when bend/turn is to the left (heading as well, so watch out when using rotation transforms)
- when the car loses traction or crashes, simulation will stop, but can be continued by clicking the RUN button
- for now, car simulation cannot handle traction loss (ie. kinetic friction)

**Road**

- the road consists of *roadSegments*, which can be straight or bend
- roadSegment length, width, radius... values are generated from random numbers (`RandGen`)
- only straight road segments can have different start and end widths, bends don't change width
- `RoadGenerator` is responsible for generating enough segments for the visible road, and accumulating the `roadLocation` along the road from travel=0.0
- `CarDriver` gets only the visible portion of the segment queue, ad the obstacle queue

**Car**

- car simulation always calculates with static friction coefficient, for now, simulation cannot handle kinetic friction
- the car is 2 wheel drive (see next point)
- the front/back weight distribution of the car is taken into account when calculating the maximum possible tangential acceleration; as only a fraction of the weight is on the driven wheels,
	max tangential acceleration will be smaller
- car turning radius from wheel angle is calculated with the Ackermann steering geometry, turn radius averaged from rear and front wheel radii
- the Car calculates the delta speed, delta odometer, delta location (in global 2D space) in every simulation iteration, and accumulates over time (`carLocation`)

**CarDriver**

- `CarDriver` first generates *road feature points* based on the generated road, and sets feature points where some action should be taken, eg.: breaking, turning for bend, accelerating for a straight segment, etc...
	Then the driving algorithm uses these featurePoints to calculate optimal actions.
- `CarDriver` handles 3 speed values:
  - *currentSpeed* is the current speed of the car
  - *cruiseSpeed* is the speed the car choses when there is no other limitation on speed (no bend, ...). This is the maximum and desirable speed of the car. You can change this on the GUI.
  - *targetSpeed* this is the current maximum speed for the car, that the driving logic calculates, eg. the maximum possible speed on e coming bend.
- acceleration calculated first, then steering, so tangent acceleration will limit max steering angle
- there are two modes of acceleration:
  - *maximum acceleration*, when the car continuously accelerates with the maximum possible tangential acceleration
  - *proportional acceleration*: the acceretation is calculated with a Proportional controller based on currentSpeed-targetSpeed error
- steering control is implemented with a PID controller, with the reference being the target car cross position, the output is the car wheelAngle. Only P and D should suffice
	(as wheel angle implictely integrates over time by the Car into the cross position, integral component is normally not needed for the steering control)
- obstacles are avoided by passing them on the side which requires the smallest crossPosition change for the car.
	If there's not enough space on this side for the car to pass the obstacle on the road, the other side is chosen


### GUI

**simulation view**

Draws a simple road, the car, and the obstacles.

Other drawn details:

- the current target crossPosition for the car steering control is drawn with a red line below the car
- speedometer shows the current speed (largest,white), the cruise speed (left bottom), and the target speed (right bottom)
- Gmeter or acceleration meter: shows the current acceleration vector of the car, scaled to the maximum allowed acceleration allowed by friction

When the car loses traction or crashes, the background color will change and the statusbar will show a message of the event.

**window docks**

- *Parameters*: here you can change some simulation / car / carDriver parameters
- *Dashboard*: shows some current values of the simulation
