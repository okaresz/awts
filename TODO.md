TODO
--------------------------

**CarDriver**

- signal something when maxTangentialAccel<epsilon? possibleCrash or sg?
- what if I'm on a turn, limited by centripAcc to brake for a coming, second turn, which comes after a straight segment on which I can brake fully? Somehow I should calculate decelDistance based on the road ahead... (not so hard..)
- (USE CARHEADINGoNROAD TO FOLLOW ROAD (setting wheelAngle at the direction of the road will pull the car always in the right direction!))
- (obstacle targetAvoidanceCrossPos is always set in every iter until obstFeaturePoint is left behind..)
- handle if there are two obstacles quickly after each other (featurePoint lookAhead)
- crossPos change on bend: if radius will be smaller, is everything recalculated? the car should slow down
- if carMinTurnRadius (current,limitedby acc) < bendRadius+epsilon -> brake so car can stay on bend
- clear up how calculations are devided between Car and CarDriver

**Simulation**

- refine/rewrite `Simulator::findTravel()`

**Visualization, GUI**

-
  
**Other**

-
