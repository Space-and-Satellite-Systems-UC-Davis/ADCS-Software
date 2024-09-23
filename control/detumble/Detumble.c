//
//  Detumble.c
//  detumbling
//
//  Created by Chandler Shellabarger on 9/21/24.
//

#include "Detumble.h"





vi_get_constant_status vi_get_mag_status()


typedef enum{
    
    DETUMBLING_SUCCESS,
    DETUMBLING_FAILIURE
    
    
    
}detumble_status;




DetumbleFunction()

{
    
    
   
    
    
    vec3 mag,lastmag;
    int deltaT;
    vec3 *angvel;
    vec3 *coilscurrent;
    int curr_millis=0,previous_millis=0;
    
    int magx,magy,magz;
    vi_get_angvel_status angvel_status;
    double angvel_x,angvel_y,angvel_z;
    vi_get_curr_millis_status time_status;
   
    
    
    
    
    
    
    time_status = vi_get_curr_millis_status(&curr_millis);
     
     if(time_status == GET_CURR_MILLIS_FAILIURE)
     {
         
         return DETUMBLING_FAILIURE;
         
     }
    
    
    
    angvel_status =  vi_get_angvel(&angvel_x,&angvel_y,&angvel_z);
     
      if(angvel_status == GET_ANGVEL_FAILIURE)
     
     {
         return DETUMBLING_FAILIURE;
     }
    
    
    
    mag.x = angvel_x;
    mag.y= angvel_y;
    mag.z=angvel_z;
    
    
    
    
    while( 0.5 < angvel_x  || 0.5 < angvel_y || angvel_z > 0.5)
    
    {
     
        lastmag = mag;

        
        previous_millis = curr_millis;
        
       time_status = vi_get_curr_millis_status(&curr_millis);
        
        if(time_status == GET_CURR_MILLIS_FAILIURE)
        {
            
            return DETUMBLING_FAILIURE;
            
        }
        
        
        angvel_status =  vi_get_angvel(&angvel_x,&angvel_y,&angvel_z);
        
         if(angvel_status == GET_ANGVEL_FAILIURE)
        
        {
            return DETUMBLING_FAILIURE;
        }
      
        
        mag.x = angvel_x;
        mag.y= angvel_y;
        mag.z=angvel_z;
        
        
        
        
        
        
        
        
        deltaT=curr_millis-previous_millis;
       
        
        
       
        
        

        
        
        bdot_control(mag,lastmag,deltaT,&coilscurrent);
        
    }
  
    
  
    
    
    
    
    return DETUMBLING_SUCCESS;
    
}
