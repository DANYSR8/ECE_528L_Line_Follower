# ECE 528/L - Robotics and Embedded Systems with Lab
**CSU Northridge**

**Department of Electrical and Computer Engineering**

## Line Follower Lab
The Line Follower lab interfaces with the following:

* User LEDs of the TI MSP432 LaunchPad
* Pololu Gearmotor with Encoder - [Product Link](https://www.pololu.com/product/3675)
* 8-Channel QTRX Sensor Array - [Product Link](https://www.pololu.com/product/3672)


## Pre Lab 

### Chassis Board LEDs Init Function 
```c

void Chassis_Board_LEDs_Init()
{
    P8->SEL0 &= ~0xE1;
    P8->SEL1 &= ~0xE1;
    P8->DIR  |= 0xE1;

    P8->OUT  &= ~0xE1;

}

```

### Print Tenth Function


```c

#include <stdio.h>
void Print_Tenth();



int main()
{
Print_Tenth();
return 0;
}



void Print_Tenth()
{

int start_count = 10; 
int end_count = 51; 

for(int i = start_count; i<51 ; i++){
    
    if(i % start_count == 0) 
    {
    printf("Index %d: Every 10th iteration.\n",i);
    } 
    
    else if(i % start_count == 1)
    {
    printf("Index %d: Following the 10th iteration.\n",i);  
    }
    
    else
    {
    printf("NA \n");    
    }
    
    }
}




```
