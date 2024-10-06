# Number Guide  
1. funnel  
2. intake  
3. flywheel  
4. swerve  
5. elevator  
6. elvator-roller  
7. pivot  

# states  
## Intake  
1. note to shooter  
2. intake  
3. stop  
4. stop  
5. idle  
6. stop  
7. intake  

## Pre-speaker  
1. flywheel-shoot  
2. intake-stop  
3. funnel-stop  
4. elevator-idle  
5. pivot-up  
6. elevator-roller stop  
7. swerve aim to speaker  

## Speaker  
1. pre speaker  
2. funnel-stop  

## Pre Amp  
1. elevator-pre-score position  
2. intake,funnel,flywheel-stop  
3. swerve-aim assist amp  

## Amp `extends` Pre Amp
elevator-score  
elevator roller-score  

## Transfer Shooter->Elevator
funnel-Shooter->Elevator  
intake-Shooter->Elevator  
...idle,stop,default   

## Transfer Elevator->Shooter
!(Shooter->Elevator)

## Idle
funel-stop  
intake-stop  
elevator-roller-stop  
flywheel-default  
swerve-default  
pivot-up  
elevator-idle  

## Outtake
funnel-outtake  
intake-outtake  
...idle,stop,default  
