//By Xinyu Ma
#ifndef GET_DISTANCE_H_INCLUDED
#define GET_DISTANCE_H_INCLUDED
/* infarade
  double get_distance() {
  return analogRead(A1));
  }
*/
#define trigPin 15
#define echoPin 12

double get_distance() {
  long duration, distance;
  digitalWrite(trigPin, LOW);  
  delayMicroseconds(2); 
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) / 29.1;
  return distance;
}
#endif // GET_DISTANCE_H_INCLUDED
