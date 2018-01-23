// This is an example of the neoPLC-BAT battery fuel-guage 
// The fuel-guage works with most single-cell LiPo and Li-Ion batteries
// Most of the algorithm is hidden in the Maxim IC, so the interface simply polls the IC for battery State of Charge (SoC)

#include <neoBAT.h>
neoBAT bat = neoBAT();

void setup() {
  
  bat.begin();

// The modelGuage algorithm automatically resets when a battery is disconnected.
// In general, battery replacement would be the only time when a reset is desired, so a software reset is never needed.
// However, for special circumstance a reset can be requested with bat.reset(); (Again, this is almost never a good idea)
  //bat.reset();

}


void loop() {

  bat.poll();
  Serial.printf("vcell = %2.4f, SoC = %2.2f%%\n\n",bat.vcell,bat.soc);

  //bat.pollAll();
  //Serial.printf("VCELL = %d, SOC = %d, VERSION = %d, CONFIG = %d\n",bat.VCELL,bat.SOC,bat.version,bat.CONFIG);
  //Serial.printf("RCOMP = 0x%2X, SLEEP = %d, ALRT = %d, ATHD = %d\n",bat.RCOMP,bat.sleepStatus,bat.alertStatus,bat.ATHD);
  //Serial.printf("vcell = %2.4f, SoC = %2.2f%%, alert threshold = %d%%\n\n",bat.vcell,bat.soc,bat.athd);
  
  delay(1000);
  
}







