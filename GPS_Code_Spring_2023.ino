#include <SoftwareSerial.h>

/**********************************************************
 LAKE LITTER SOLUTIONS GPS MODULE
************************************************************/

#include <SoftwareSerial.h>
#include <TinyGPS.h>

#define RXpin 9
#define TXpin 8

SoftwareSerial mySerial(TXpin,RXpin);
TinyGPS gps;

void printFloat(double f, int digits = 2);

void setup() {
  
  Serial.begin(9600);
  mySerial.begin(9600);
  delay(100);
  
}

void loop() 
{
  bool newdata = false;
  unsigned long start = millis();

  while(millis() - start < 5000)
  {
    if(mySerial.available())
    {
      char c = mySerial.read();
      if(gps.encode(c)) //this is what i think will make newdata true, something might not be working for this if to be reached
      {
        newdata = true;
      }
    }
  }
    
  if(newdata) //if newdata is true, the info is printed 
  {
    Serial.println("Acquired Data: ");
    Serial.println("-------------------------------------------");
    gpsdata(gps);
    Serial.println("\n-------------------------------------------");
    Serial.println();
    Serial.println("gps test test test");
  }

  Serial.println("naur neweur dataeur"); //default check, prints no matter what. if this is the only thing printed, then gps is not checked (newdata remains false)
}
     
void gpsdata(TinyGPS &gps)
{
  long latitude, longitude;
  float flatitude,flongitude;
  unsigned long date, time, chars, age;
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned short sentences, failed;

  //latitude and longitude in 10^-5deg
  gps.get_position(&latitude, &longitude, &age);
  Serial.print("Lat/Long(float): "); Serial.print(latitude);Serial.print(", "); Serial.print(longitude);
  Serial.print(" Fix age: "); Serial.print(age); Serial.print("ms.");
  Serial.print("\n"); 

  //latitude and longitude in float
  gps.f_get_position(&flatitude, &flongitude, &age);
  Serial.print("Lat/Long(float): "); printFloat(flatitude,5);Serial.print(", "); printFloat(flongitude,5);
  Serial.print(" Fix age: "); Serial.print(age); Serial.print("ms.");
  Serial.print("\n"); 

  //get time and date
  gps.get_datetime(&date, &time, &age);
  Serial.print("Date(ddmmyy): "); Serial.print(date); Serial.print(" Time(hhmmsscc): "); Serial.print(time);
  Serial.print(" Fix age: "); Serial.print(age); Serial.print("ms.");
  Serial.print("\n"); 

  //time and date in UTC +8 Malaysia
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  Serial.print("Date: "); Serial.print(static_cast<int>(month)); Serial.print("/"); 
  Serial.print(static_cast<int>(day)); Serial.print("/"); Serial.print(year);
  Serial.print("  Time: "); Serial.print(static_cast<int>(hour+8));  Serial.print(":"); //Serial.print("UTC +08:00 Malaysia");
  Serial.print(static_cast<int>(minute)); Serial.print(":"); Serial.print(static_cast<int>(second));
  Serial.print("."); Serial.print(static_cast<int>(hundredths)); Serial.print(" UTC +08:00 Malaysia");
  Serial.print(" Fix age: "); Serial.print(age); Serial.print("ms.");
  Serial.print("\n"); 

  //get altitude and wind speed
  Serial.print("Alt(cm): "); Serial.print(gps.altitude()); Serial.print(" Course(10^-2 deg): ");Serial.print(gps.course()); Serial.print(" Speed(10^-2 knots): "); Serial.println(gps.speed());
  Serial.print("Alt(float): "); printFloat(gps.f_altitude()); Serial.print(" Course(float): ");printFloat(gps.f_course()); Serial.println();
  Serial.print("Speed(knots): "); printFloat(gps.f_speed_knots()); Serial.print(" (mph): ");printFloat(gps.f_speed_mph());
  Serial.print(" (mps): "); printFloat(gps.f_speed_mps()); Serial.print(" (kmph): ");printFloat(gps.f_speed_kmph()); 
  Serial.print("\n"); 

  //get statistics
  gps.stats(&chars, &sentences, &failed);
  Serial.print("Stats: characters: "); Serial.print(chars); Serial.print(" sentences: ");
  Serial.print(sentences); Serial.print(" failed checksum: "); Serial.print(failed);
}

void printFloat(double number, int digits)
{
  // Handle negative numbers
  if (number < 0.0) 
  {
     Serial.print('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  Serial.print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    Serial.print("."); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0) 
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint;
  }
}
