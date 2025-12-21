mdp //Markov decision process

// this Model represent a Markov decision process for 2 clsuters consists of tens of blocks
//Labels
label "failed" = (s1=task_failed) | (s2=task_failed);
label "success" = (s1=finish) & (s2=finish);
label "both_active" = active1 & active2;
label "high_energy" = (s1=high_blink_high_buzz | s1=high_Led | s1=high_pitch) | 
                     (s2=high_blink_high_buzz | s2=high_Led | s2=high_pitch);
label "noHWEnv" =
  (hw_state=0 & sound=0 & move=0 & env_state1=0 & env_state2=0);
label "low_energy" = (s1=med_blink_med_buzz | s1=low_Led | s1=low_pitch) | 
                     (s2=med_blink_med_buzz | s2=low_Led | s2=low_pitch);
label "LED_color" = (color1!=0) | (color2!=0) ;
//////

// Constants for states
const int start_task = 0;
const int choose_Red = 1;
const int choose_Blue = 2;
const int choose_white = 3;
const int listen_on_moving_on = 4;
const int listen_on_moving_off = 5;
const int listen_off_moving_on = 6;

const int high_pitch = 7;
const int med_pitch = 8;
const int low_pitch = 9;
const int high_blink_high_buzz = 10;
const int high_blink_med_buzz = 11;
const int med_blink_high_buzz = 12;
const int med_blink_med_buzz = 13;
const int high_Led = 14;
const int med_Led = 15;
const int low_Led = 16;

const int task_failed = 17;
const int finish = 18;
const int idle = 19;// think on how to put on task controller
// Base probability for line cluster
//patterns coefficient
const double theta;// this is a ratio that in rectangle it manage to runs more blocks that in aline pattern by this value when the same power supply spot(edged)
const double beta1;//=1.022;// is the coefficient of success from high to medium state;where beta<1.05 & beta >1 ---1.022 in this case
const double beta2;//=1.05;// is the coefficient of success from high to low state ;where beta<1.1 & beta >1.05 ---1.05 in this case
const double gamma;// coefficient for staggering against sparse transitions.

const double alpha2;   // Cluster 2 related to cluster 1 since distance increase failure probability  

// base success prob for high_LED in cluster 1 (per color)
const double q_w_high_c1;//=0.7; // white, cluster1
const double q_b_high_c1;//=0.8; // blue,  cluster1
const double q_r_high_c1;//=0.9; // red,   cluster1
const double q_high_pitch_c1;//=0.9; // highpitch, cluster1 (assumed)

module Operator
staggered:bool init false;//means it is concurrently working
[staggered] staggered=false & s1=idle & s2=idle ->(staggered'=true);
endmodule

module pattern
Rec_motif1:bool init false;//means it is line pattern
Rec_motif2:bool init false;//means it is line pattern
//Rec_motif3:bool init false;//means it is line pattern
[Rec_1]Rec_motif1=false & s1=idle & task_counter=0  ->(Rec_motif1'=true);
[Rec_2]Rec_motif2=false & s2=idle & task_counter=0->(Rec_motif2'=true);
//[Rec_3]Rec_motif3=false & s3=0->(Rec_motif3'=true);
endmodule


module Hardware
  hw_state : [0..2] init 0;

  // Can be updated via some sync or monitoring
  [update_hw_med] hw_state=0 & task_counter<=2 & (s1> listen_off_moving_on & s1< task_failed) & (s2> listen_off_moving_on & s2< task_failed) -> (hw_state'=1);
  [update_hw_high] hw_state=0 & task_counter<=2 & (s1> listen_off_moving_on & s1< task_failed) & (s2> listen_off_moving_on & s2< task_failed)-> (hw_state'=2);
  [reset_hw] hw_state>0& task_counter<=2  &  (s1> task_failed & s2> task_failed) -> (hw_state'=0);

endmodule

// Environment sensing: 0 = idle, 1 = MIC, 2 = ACC, 3 = BOTH
module Environment
  sound :[0..1] init 0;
  move :[0..1] init 0;
  env_state1 : [0..3] init 0;
  env_state2 : [0..3] init 0;

  [sound_on] sound=0 & task_counter<2 &s1!=start_task & (env_state1=1)-> (sound'=1);
  [move_on] move=0 & task_counter<2  &s1!=start_task -> (move'=1);
  [sound_off] sound=1 & task_counter<2  &s1!=start_task -> (sound'=0);
  [move_off] move=1& task_counter<2  &s1!=start_task-> (move'=0);
  [sense_mic1] s1!=start_task & env_state1!=1  -> (env_state1'=1);
  [sense_acc1] s1!=start_task & env_state1!=2 -> (env_state1'=2);
  [sense_both1]s1!=start_task & env_state1!=3 -> (env_state1'=3);

  [sense_mic2] s2!=start_task & env_state2!=1 -> (env_state2'=1);
  [sense_acc2] s2!=start_task & env_state2!=2  -> (env_state2'=2);
  [sense_both2]s2!=start_task & env_state2!=3  -> (env_state2'=3);
  
// add more syncing action per module yet after detecting you might take action upon the full system or
//or partially per module
  [reset_env1] env_state1!=0  & (s1=finish | s1=task_failed) -> (env_state1'=0);
  [reset_env2] env_state2!=0 & (s2=finish | s2=task_failed)-> (env_state2'=0);

endmodule

//cluster sizes
const int N1 = 10;
const int N2 = 10;

module Task_Controller
  task_counter : [0..2] init 0;
  active1 : bool init true;
  active2 : bool init true;
  turn: [0..1] init 0;// depends on number of clusters in here 2 modules

  [activate_1] (s1=idle) & !active1 & task_counter=1 -> (active1'=true);
  [activate_2] (s1=idle&s2=idle) & !active2 & task_counter=1 -> (active2'=true);
  

  [deactivate_1] (s1=idle) & active1 -> (active1'=false);
  [deactivate_2] (s1=idle&s2=idle) & active2 -> (active2'=false);

   [Task_increment] task_counter<2 & 
        ( (!active1 | (active1  & (s1=finish | s1=task_failed))) &
         (!active2 | (active2  & (s2=finish | s2=task_failed))))  ->
         (task_counter'=task_counter+1);

[turn1] turn =0 & (s1=finish|s1=task_failed) & staggered=true-> (turn'=1);
//[turn2] turn =1 & (s2=finish|s2=task_failed) & staggered=true-> (turn'=0);
endmodule

// BB1: Cluster 1 of Blinky Blocks
module c1
  s1 : [0..19] init idle;
  color1 : [0..3] init 0;
//to reduce state explosion and to avoid non determinism from the beginning of the model
  [] (s1=idle) -> (s1'=start_task) ;
// dummy transition
  [step] (s1=start_task)& !active1 -> true ;

  // Start task (guarded by active1)
  [step] s1=start_task & active1 -> (s1'=choose_Red) & (color1'=1) ;
  [step] s1=start_task & active1 -> (s1'=choose_Blue) & (color1'=2) ;
  [step] s1=start_task & active1 -> (s1'=choose_white) & (color1'=3) ;
  [step] s1=start_task & active1 -> (s1'=high_pitch) ;
  [step] s1=start_task & active1 -> (s1'=med_pitch) ;
  [step] s1=start_task & active1 -> (s1'=low_pitch) ;
  [sense_both1] s1=start_task & active1 -> (s1'=listen_on_moving_on) ;
  [sense_mic1] s1=start_task & active1 -> (s1'=listen_on_moving_off) ;
  [sense_acc1] s1=start_task & active1 -> (s1'=listen_off_moving_on) ;

  // Color selection
  [step] s1=choose_Red -> (s1'=high_blink_high_buzz)  ;
  [step] s1=choose_Red -> (s1'=high_blink_med_buzz) ;
  [step] s1=choose_Red -> (s1'=med_blink_high_buzz) ;
  [step] s1=choose_Red -> (s1'=med_blink_med_buzz) ;
  [step] s1=choose_Red -> (s1'=high_Led) ;
  [step] s1=choose_Red -> (s1'=med_Led) ;
  [step] s1=choose_Red -> (s1'=low_Led) ;

  [step] s1=choose_Blue -> (s1'=high_blink_high_buzz) ;
  [step] s1=choose_Blue -> (s1'=high_blink_med_buzz) ;
  [step] s1=choose_Blue -> (s1'=med_blink_high_buzz) ;
  [step] s1=choose_Blue -> (s1'=med_blink_med_buzz) ;
  [step] s1=choose_Blue -> (s1'=high_Led) ;
  [step] s1=choose_Blue -> (s1'=med_Led) ;
  [step] s1=choose_Blue -> (s1'=low_Led) ;

  [step] s1=choose_white -> (s1'=high_blink_high_buzz) ;
  [step] s1=choose_white -> (s1'=high_blink_med_buzz) ;
  [step] s1=choose_white -> (s1'=med_blink_high_buzz) ;
  [step] s1=choose_white -> (s1'=med_blink_med_buzz) ;
  [step] s1=choose_white -> (s1'=high_Led) ;
  [step] s1=choose_white -> (s1'=med_Led) ;
  [step] s1=choose_white -> (s1'=low_Led) ;
////////////////////////////////////
  // Probabilistic transitions (BB1-specific) LINE pattern syncroninzed with other clusters
///////////////////////////////////
//high pitch
  [step] s1=high_pitch & hw_state=2 & Rec_motif1=false & staggered=false -> (s1'=low_pitch); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_pitch & hw_state=1 & Rec_motif1=false & staggered=false -> (s1'=med_pitch);// Priority 2: hardware MED -> reduce one level
  [step] s1=high_pitch & hw_state=0 & env_state1=2  & move=1& Rec_motif1=false & staggered=false ->(1-q_high_pitch_c1)/2 : (s1'=med_pitch) + 1-((1-q_high_pitch_c1)/2) : (s1'=low_pitch);// Priority 3: environment sensing
  [step] s1=high_pitch & hw_state=0 & env_state1= 0 & move=1& Rec_motif1=false & staggered=false->(1-q_high_pitch_c1) : (s1'=task_failed)+ q_high_pitch_c1/2 : (s1'=med_pitch) + q_high_pitch_c1/2 : (s1'=low_pitch);// when move (no direct intention of fallback), chance of failure occurs
  [step] s1=high_pitch & hw_state=0 & env_state1=1  & sound=1 & Rec_motif1=false & staggered=false->1-((1-q_high_pitch_c1)/2) : (s1'=med_pitch) + (1-q_high_pitch_c1)/2 : (s1'=low_pitch);
  [step] s1=high_pitch& hw_state=0   & (sound=0 & move=0) & Rec_motif1=false & staggered=false-> q_high_pitch_c1  : (s1'=finish) + (1-q_high_pitch_c1)/2 : (s1'=med_pitch) + (1-q_high_pitch_c1)/2 : (s1'=low_pitch); // default

//med pitch
  [step] s1=med_pitch & (hw_state=2 |hw_state=1) & Rec_motif1=false & staggered=false -> (s1'=low_pitch); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [step] s1=med_pitch & hw_state=0 & (env_state1=2 |env_state1=1)  &( sound=1 |move=1)& Rec_motif1=false & staggered=false -> (s1'=low_pitch);// Priority 3: environment sensing
  [step] s1=med_pitch & hw_state=0 & env_state1= 0 & move=1& Rec_motif1=false & staggered=false-> 1-(q_high_pitch_c1*beta1) : (s1'=task_failed)+ q_high_pitch_c1*beta1 : (s1'=low_pitch);// when move (no direct intention of fallback), chance of failure occurs
  [step] s1=med_pitch& hw_state=0  & (sound=0 & move=0)& Rec_motif1=false & staggered=false-> q_high_pitch_c1*beta1 : (s1'=finish) + (1-(q_high_pitch_c1*beta1)) : (s1'=low_pitch);// default

//low pitch
  [step] s1=low_pitch & (hw_state=2 |env_state1=2)& Rec_motif1=false & staggered=false -> (1-(q_high_pitch_c1*beta2)) : (s1'=finish) + q_high_pitch_c1*beta2 : (s1'=task_failed); 
  [step] s1=low_pitch & (hw_state=1 |env_state1=1)& Rec_motif1=false & staggered=false -> (1-((q_high_pitch_c1*beta2)/2)) : (s1'=finish) + (q_high_pitch_c1*beta2)/2 : (s1'=task_failed); 
  [step] s1=low_pitch & hw_state=0 & Rec_motif1=false & staggered=false-> q_high_pitch_c1*beta2 : (s1'=finish) + (1-(q_high_pitch_c1*beta2)) : (s1'=task_failed);//even if movement it is already very low consuming and it is the lowest state so failure exist but not as when a higher consuming state

//sensor-environment transition
  [step] s1=listen_on_moving_on & Rec_motif1=false & staggered=false-> 0.95 : (s1'=finish) + 0.05 : (s1'=task_failed);
  [step] s1=listen_on_moving_off & Rec_motif1=false & staggered=false-> 0.95 : (s1'=finish) + 0.05 : (s1'=task_failed);
  [step] s1=listen_off_moving_on& Rec_motif1=false & staggered=false -> 0.95 : (s1'=finish) + 0.05 : (s1'=task_failed);

//high-high simultaneous actuating white
  [step] s1=high_blink_high_buzz &color1=3 & hw_state=2 & Rec_motif1=false & staggered=false -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_blink_high_buzz &color1=3 & hw_state=1 & Rec_motif1=false & staggered=false -> 0.5 : (s1'=high_blink_med_buzz) +0.5: (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level
  [step] s1=high_blink_high_buzz &color1=3 & hw_state=0 & env_state1=2  & move=1& Rec_motif1=false & staggered=false->0.5 : (s1'=med_blink_high_buzz) + 0.5 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=high_blink_high_buzz &color1=3 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=false & staggered=false->1-(q_w_high_c1*q_high_pitch_c1) : (s1'=task_failed)+(q_w_high_c1*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + (q_w_high_c1*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s1=high_blink_high_buzz &color1=3 & hw_state=0 & env_state1=1  & sound=1& Rec_motif1=false & staggered=false->q_w_high_c1*q_high_pitch_c1 : (s1'=high_blink_med_buzz) + 1-(q_w_high_c1*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [step] s1=high_blink_high_buzz &color1=3 & hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=false & staggered=false -> q_w_high_c1*q_high_pitch_c1: (s1'=finish) + (1-(q_w_high_c1*q_high_pitch_c1))/3 : (s1'=high_blink_med_buzz) + (1-(q_w_high_c1*q_high_pitch_c1))/3 : (s1'=med_blink_high_buzz) + (1-(q_w_high_c1*q_high_pitch_c1))/3 : (s1'=med_blink_med_buzz); //default(Empirical)
//high-high simultaneous actuating blue
  [step] s1=high_blink_high_buzz &color1=2 & hw_state=2 & Rec_motif1=false & staggered=false -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_blink_high_buzz &color1=2 & hw_state=1 & Rec_motif1=false & staggered=false -> 0.5 : (s1'=high_blink_med_buzz) +0.5: (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level
  [step] s1=high_blink_high_buzz &color1=2 & hw_state=0 & env_state1=2  & move=1& Rec_motif1=false & staggered=false->0.5 : (s1'=med_blink_high_buzz) + 0.5 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=high_blink_high_buzz &color1=2 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=false & staggered=false->1-(q_b_high_c1*q_high_pitch_c1) : (s1'=task_failed)+(q_b_high_c1*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + (q_b_high_c1*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s1=high_blink_high_buzz &color1=2 & hw_state=0 & env_state1=1  & sound=1& Rec_motif1=false & staggered=false->q_b_high_c1*q_high_pitch_c1 : (s1'=high_blink_med_buzz) + 1-(q_b_high_c1*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [step] s1=high_blink_high_buzz &color1=2 & hw_state=0 & env_state1=0  & (sound=0 & move=0) & Rec_motif1=false & staggered=false-> q_b_high_c1*q_high_pitch_c1: (s1'=finish) + (1-(q_b_high_c1*q_high_pitch_c1))/3 : (s1'=high_blink_med_buzz) + (1-(q_b_high_c1*q_high_pitch_c1))/3 : (s1'=med_blink_high_buzz) + (1-(q_b_high_c1*q_high_pitch_c1))/3 : (s1'=med_blink_med_buzz); //default(Empirical)
//high-high simultaneous actuating RED
  [step] s1=high_blink_high_buzz &color1=1 & hw_state=2 & Rec_motif1=false & staggered=false -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_blink_high_buzz &color1=1 & hw_state=1 & Rec_motif1=false & staggered=false -> 0.5 : (s1'=high_blink_med_buzz) +0.5: (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level
  [step] s1=high_blink_high_buzz &color1=1 & hw_state=0 & env_state1=2  & move=1& Rec_motif1=false & staggered=false->0.5 : (s1'=med_blink_high_buzz) + 0.5 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=high_blink_high_buzz &color1=1 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=false & staggered=false->1-(q_r_high_c1*q_high_pitch_c1) : (s1'=task_failed)+(q_r_high_c1*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + (q_r_high_c1*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s1=high_blink_high_buzz &color1=1 & hw_state=0 & env_state1=1  & sound=1& Rec_motif1=false & staggered=false->q_r_high_c1*q_high_pitch_c1 : (s1'=high_blink_med_buzz) + 1-(q_r_high_c1*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [step] s1=high_blink_high_buzz &color1=1 & hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=false & staggered=false -> q_r_high_c1*q_high_pitch_c1: (s1'=finish) + (1-(q_r_high_c1*q_high_pitch_c1))/3 : (s1'=high_blink_med_buzz) + (1-(q_r_high_c1*q_high_pitch_c1))/3 : (s1'=med_blink_high_buzz) + (1-(q_r_high_c1*q_high_pitch_c1))/3 : (s1'=med_blink_med_buzz); //default(Empirical)

//high-med simultaneous actuating white
  [step] s1=high_blink_med_buzz &color1=3 & hw_state=2& Rec_motif1=false & staggered=false  -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_blink_med_buzz &color1=3 & hw_state=1 & Rec_motif1=false & staggered=false -> 0.5 : (s1'=med_blink_med_buzz) + 0.5 : (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level /equal probabilities cause it deoends on user preferences
  [step] s1=high_blink_med_buzz &color1=3& hw_state=0 & env_state1=2  & move=1& Rec_motif1=false & staggered=false->1-(q_w_high_c1*q_high_pitch_c1) : (s1'=med_blink_high_buzz) + q_w_high_c1*q_high_pitch_c1 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=high_blink_high_buzz &color1=3 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=false & staggered=false->1-(q_w_high_c1*q_high_pitch_c1) : (s1'=task_failed)+(q_w_high_c1*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + (q_w_high_c1*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s1=high_blink_med_buzz &color1=3& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=false & staggered=false->q_w_high_c1*q_high_pitch_c1  : (s1'=med_blink_med_buzz) + 1-(q_w_high_c1*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [step] s1=high_blink_med_buzz &color1=3& hw_state=0 & env_state1=0  & (sound=0 & move=0) & Rec_motif1=false & staggered=false-> q_w_high_c1*q_high_pitch_c1 : (s1'=finish)  + (1-(q_w_high_c1*q_high_pitch_c1))/2 : (s1'=med_blink_high_buzz) + (1-(q_w_high_c1*q_high_pitch_c1))/2 : (s1'=med_blink_med_buzz); //default(empirical)
 //high-med simultaneous actuating blue
   [step] s1=high_blink_med_buzz &color1=2 & hw_state=2 & Rec_motif1=false & staggered=false -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_blink_med_buzz &color1=2 & hw_state=1 & Rec_motif1=false & staggered=false -> 0.5 : (s1'=med_blink_med_buzz) + 0.5 : (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level /equal probabilities cause it deoends on user preferences
  [step] s1=high_blink_med_buzz &color1=2& hw_state=0 & env_state1=2  & move=1& Rec_motif1=false & staggered=false->1-(q_b_high_c1*q_high_pitch_c1) : (s1'=med_blink_high_buzz) + q_b_high_c1*q_high_pitch_c1 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=high_blink_high_buzz &color1=2 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=false & staggered=false->1-(q_b_high_c1*q_high_pitch_c1) : (s1'=task_failed)+(q_b_high_c1*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + (q_b_high_c1*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s1=high_blink_med_buzz &color1=2& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=false & staggered=false->q_b_high_c1*q_high_pitch_c1  : (s1'=med_blink_med_buzz) + 1-(q_b_high_c1*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [step] s1=high_blink_med_buzz &color1=2& hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=false & staggered=false -> q_b_high_c1*q_high_pitch_c1 : (s1'=finish)  + (1-(q_b_high_c1*q_high_pitch_c1))/2 : (s1'=med_blink_high_buzz) + (1-(q_b_high_c1*q_high_pitch_c1))/2 : (s1'=med_blink_med_buzz); //default(empirical)
 
//high-med simultaneous actuating Red
  [step] s1=high_blink_med_buzz &color1=1 & hw_state=2 & Rec_motif1=false & staggered=false -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_blink_med_buzz &color1=1 & hw_state=1 & Rec_motif1=false & staggered=false -> 0.5 : (s1'=med_blink_med_buzz) + 0.5 : (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level /equal probabilities cause it deoends on user preferences
  [step] s1=high_blink_med_buzz &color1=1& hw_state=0 & env_state1=2  & move=1& Rec_motif1=false & staggered=false->1-(q_r_high_c1*q_high_pitch_c1) : (s1'=med_blink_high_buzz) + q_r_high_c1*q_high_pitch_c1 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=high_blink_high_buzz &color1=1 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=false & staggered=false->1-(q_r_high_c1*q_high_pitch_c1) : (s1'=task_failed)+(q_r_high_c1*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + (q_r_high_c1*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s1=high_blink_med_buzz &color1=1& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=false & staggered=false->q_r_high_c1*q_high_pitch_c1  : (s1'=med_blink_med_buzz) + 1-(q_r_high_c1*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [step] s1=high_blink_med_buzz &color1=1& hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=false & staggered=false -> q_r_high_c1*q_high_pitch_c1 : (s1'=finish)  + (1-(q_r_high_c1*q_high_pitch_c1))/2 : (s1'=med_blink_high_buzz) + (1-(q_r_high_c1*q_high_pitch_c1))/2 : (s1'=med_blink_med_buzz); //default(empirical)
 

 //med-high simultaneous actuating white
  [step] s1=med_blink_high_buzz &color1=3 & (hw_state=2 | hw_state=1) & Rec_motif1=false & staggered=false -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=med_blink_high_buzz &color1=3 & hw_state=0 & (env_state1=2 | env_state1=1)   & (sound=1 | move=1)& Rec_motif1=false & staggered=false-> (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=med_blink_high_buzz &color1=3 & hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=false & staggered=false-> q_w_high_c1*q_high_pitch_c1*beta1 : (s1'=finish)  + 1-(q_w_high_c1*q_high_pitch_c1*beta1) : (s1'=med_blink_med_buzz); //default(empirical)
  [step] s1=med_blink_high_buzz &color1=3 & hw_state=0 & env_state1=0  & ( move=1)& Rec_motif1=false & staggered=false-> 1-(q_w_high_c1*q_high_pitch_c1*beta1) : (s1'=task_failed)  + q_w_high_c1*q_high_pitch_c1*beta1 : (s1'=med_blink_med_buzz); //when movement un-intentionally
 //med-high simultaneous actuating blue
  [step] s1=med_blink_high_buzz &color1=2 & (hw_state=2 | hw_state=1)& Rec_motif1=false & staggered=false  -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=med_blink_high_buzz &color1=2 & hw_state=0 & (env_state1=2 | env_state1=1)   & (sound=1 | move=1)& Rec_motif1=false & staggered=false-> (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=med_blink_high_buzz &color1=2 & hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=false & staggered=false-> q_b_high_c1*q_high_pitch_c1*beta1 : (s1'=finish)  + 1-(q_b_high_c1*q_high_pitch_c1*beta1) : (s1'=med_blink_med_buzz); //default(empirical)
  [step] s1=med_blink_high_buzz &color1=2 & hw_state=0 & env_state1=0  & ( move=1)& Rec_motif1=false & staggered=false-> 1-(q_b_high_c1*q_high_pitch_c1*beta1) : (s1'=task_failed)  + q_b_high_c1*q_high_pitch_c1*beta1 : (s1'=med_blink_med_buzz); //when movement un-intentionally
 //med-high simultaneous actuating Red
  [step] s1=med_blink_high_buzz &color1=1 & (hw_state=2 | hw_state=1) & Rec_motif1=false & staggered=false -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=med_blink_high_buzz &color1=1 & hw_state=0 & (env_state1=2 | env_state1=1)   & (sound=1 | move=1)& Rec_motif1=false & staggered=false-> (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=med_blink_high_buzz &color1=1 & hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=false & staggered=false-> q_r_high_c1*q_high_pitch_c1*beta1 : (s1'=finish)  + 1-(q_r_high_c1*q_high_pitch_c1*beta1) : (s1'=med_blink_med_buzz); //default(empirical)
  [step] s1=med_blink_high_buzz &color1=1 & hw_state=0 & env_state1=0  & ( move=1) & Rec_motif1=false & staggered=false-> 1-(q_r_high_c1*q_high_pitch_c1*beta1) : (s1'=task_failed)  + q_r_high_c1*q_high_pitch_c1*beta1 : (s1'=med_blink_med_buzz); //when movement un-intentionally



//med-med simultaneous actuating white
  [step] s1=med_blink_med_buzz &color1=3& (hw_state=2 |env_state1=2)& Rec_motif1=false & staggered=false -> 1-(q_w_high_c1*q_high_pitch_c1*beta2) : (s1'=finish) + (q_w_high_c1*q_high_pitch_c1*beta2) : (s1'=task_failed); 
  [step] s1=med_blink_med_buzz &color1=3& (hw_state=1 |env_state1=1) & Rec_motif1=false & staggered=false-> 0.5 : (s1'=finish) + 0.5 : (s1'=task_failed); 
  [step] s1=med_blink_med_buzz &color1=3& hw_state=0& Rec_motif1=false & staggered=false-> (q_w_high_c1*q_high_pitch_c1*beta2) : (s1'=finish) + 1-(q_w_high_c1*q_high_pitch_c1*beta2) : (s1'=task_failed);//empirical data
//med-med simultaneous actuating Blue
[step] s1=med_blink_med_buzz &color1=2& (hw_state=2 |env_state1=2) & Rec_motif1=false & staggered=false-> 1-(q_b_high_c1*q_high_pitch_c1*beta2) : (s1'=finish) + (q_b_high_c1*q_high_pitch_c1*beta2) : (s1'=task_failed); 
  [step] s1=med_blink_med_buzz &color1=2& (hw_state=1 |env_state1=1)& Rec_motif1=false & staggered=false -> 0.5 : (s1'=finish) + 0.5 : (s1'=task_failed); 
  [step] s1=med_blink_med_buzz &color1=2& hw_state=0& Rec_motif1=false & staggered=false-> (q_b_high_c1*q_high_pitch_c1*beta2) : (s1'=finish) + 1-(q_b_high_c1*q_high_pitch_c1*beta2) : (s1'=task_failed);//empirical data
//med-med simultaneous actuating Red
  [step] s1=med_blink_med_buzz &color1=1& (hw_state=2 |env_state1=2)& Rec_motif1=false & staggered=false -> 1-(q_r_high_c1*q_high_pitch_c1*beta2) : (s1'=finish) + (q_r_high_c1*q_high_pitch_c1*beta2) : (s1'=task_failed); 
  [step] s1=med_blink_med_buzz &color1=1& (hw_state=1 |env_state1=1) & Rec_motif1=false & staggered=false-> 0.5 : (s1'=finish) + 0.5 : (s1'=task_failed); 
  [step] s1=med_blink_med_buzz &color1=1& hw_state=0 & Rec_motif1=false & staggered=false-> (q_r_high_c1*q_high_pitch_c1*beta2) : (s1'=finish) + 1-(q_r_high_c1*q_high_pitch_c1*beta2) : (s1'=task_failed);//empirical data

//High LED white
  [step] s1=high_Led  &color1=3& hw_state=2 & Rec_motif1=false & staggered=false -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_Led  &color1=3& hw_state=1 & Rec_motif1=false & staggered=false -> (s1'=med_Led);// Priority 2: hardware MED -> reduce one level
  [step] s1=high_Led  &color1=3& hw_state=0 & env_state1=2 & move=1& Rec_motif1=false & staggered=false->0.5  : (s1'=med_Led) + 0.5 : (s1'=low_Led);// Priority 3: environment sensing
  [step] s1=high_Led  &color1=3& hw_state=0 & env_state1=0 & move=1& Rec_motif1=false & staggered=false->(1-q_w_high_c1) : (s1'=task_failed)  +q_w_high_c1/2  : (s1'=med_Led) + q_w_high_c1/2 : (s1'=low_Led);// when movement un-intentionally
  [step] s1=high_Led  &color1=3& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=false & staggered=false->0.5 : (s1'=med_Led) + 0.5 : (s1'=low_Led);
  [step] s1=high_Led  &color1=3& hw_state=0 & env_state1=0 & (sound=0 & move=0) & Rec_motif1=false & staggered=false-> q_w_high_c1 : (s1'=finish) + (1-q_w_high_c1)/2 : (s1'=med_Led) + (1-q_w_high_c1)/2 : (s1'=low_Led); // default (empirical)
//High LED Blue
  [step] s1=high_Led  &color1=2& hw_state=2 & Rec_motif1=false & staggered=false -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_Led  &color1=2& hw_state=1 & Rec_motif1=false & staggered=false -> (s1'=med_Led);// Priority 2: hardware MED -> reduce one level
  [step] s1=high_Led  &color1=2& hw_state=0 & env_state1=2 & move=1& Rec_motif1=false & staggered=false->0.5  : (s1'=med_Led) + 0.5 : (s1'=low_Led);// Priority 3: environment sensing
  [step] s1=high_Led  &color1=2& hw_state=0 & env_state1=0 & move=1& Rec_motif1=false & staggered=false->(1-q_b_high_c1) : (s1'=task_failed)  +q_b_high_c1/2  : (s1'=med_Led) + q_b_high_c1/2 : (s1'=low_Led);// when movement un-intentionally
  [step] s1=high_Led  &color1=2& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=false & staggered=false->0.5 : (s1'=med_Led) + 0.5 : (s1'=low_Led);
  [step] s1=high_Led  &color1=2& hw_state=0 & env_state1=0 & (sound=0 & move=0)& Rec_motif1=false & staggered=false -> q_b_high_c1 : (s1'=finish) + (1-q_b_high_c1)/2 : (s1'=med_Led) + (1-q_b_high_c1)/2 : (s1'=low_Led); // default (empirical)
//High LED Red
   [step] s1=high_Led  &color1=1& hw_state=2 & Rec_motif1=false & staggered=false -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_Led  &color1=1& hw_state=1 & Rec_motif1=false & staggered=false -> (s1'=med_Led);// Priority 2: hardware MED -> reduce one level
  [step] s1=high_Led  &color1=1& hw_state=0 & env_state1=2 & move=1& Rec_motif1=false & staggered=false->0.5  : (s1'=med_Led) + 0.5 : (s1'=low_Led);// Priority 3: environment sensing
  [step] s1=high_Led  &color1=1& hw_state=0 & env_state1=0 & move=1& Rec_motif1=false & staggered=false->(1-q_r_high_c1) : (s1'=task_failed)  +q_r_high_c1/2  : (s1'=med_Led) + q_r_high_c1/2 : (s1'=low_Led);// when movement un-intentionally
  [step] s1=high_Led  &color1=1& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=false & staggered=false->0.5 : (s1'=med_Led) + 0.5 : (s1'=low_Led);
  [step] s1=high_Led  &color1=1& hw_state=0 & env_state1=0 & (sound=0 & move=0)& Rec_motif1=false & staggered=false -> q_r_high_c1 : (s1'=finish) + (1-q_r_high_c1)/2 : (s1'=med_Led) + (1-q_r_high_c1)/2 : (s1'=low_Led); // default (empirical)

//Med LED white
  [step] s1=med_Led  &color1=3& (hw_state=2 |hw_state=1) & Rec_motif1=false & staggered=false -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [step] s1=med_Led  &color1=3& hw_state=0 & (env_state1=2 |env_state1=1)  & (sound=1 | move=1)& Rec_motif1=false & staggered=false-> (s1'=low_Led);// Priority 3: environment sensing
  [step] s1=med_Led  &color1=3& hw_state=0 & (sound=0 | move=0)& Rec_motif1=false & staggered=false-> q_w_high_c1*beta1 : (s1'=finish) + 1-(q_w_high_c1*beta1) : (s1'=low_Led);// default (empirical)
  [step] s1=med_Led  &color1=3& hw_state=0 & (move=1)& Rec_motif1=false & staggered=false-> 1-(q_w_high_c1*beta1) : (s1'=task_failed) + q_w_high_c1*beta1 : (s1'=low_Led);// when movement un-intentionally
//Med LED Blue
  [step] s1=med_Led   &color1=2& (hw_state=2 |hw_state=1) & Rec_motif1=false & staggered=false -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [step] s1=med_Led  &color1=2& hw_state=0 & (env_state1=2 |env_state1=1)  & (sound=1 | move=1)& Rec_motif1=false & staggered=false-> (s1'=low_Led);// Priority 3: environment sensing
  [step] s1=med_Led  &color1=2& hw_state=0 & (sound=0 | move=0)& Rec_motif1=false & staggered=false-> q_b_high_c1*beta1 : (s1'=finish) + 1-(q_b_high_c1*beta1) : (s1'=low_Led);// default (empirical)
  [step] s1=med_Led  &color1=2& hw_state=0 & (move=1)& Rec_motif1=false & staggered=false-> 1-(q_b_high_c1*beta1) : (s1'=task_failed) + q_b_high_c1*beta1 : (s1'=low_Led);// when movement un-intentionally
//Med LED Red
  [step] s1=med_Led   &color1=1& (hw_state=2 |hw_state=1) & Rec_motif1=false & staggered=false -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [step] s1=med_Led  &color1=1& hw_state=0 & (env_state1=2 |env_state1=1)  & (sound=1 | move=1)& Rec_motif1=false & staggered=false-> (s1'=low_Led);// Priority 3: environment sensing
  [step] s1=med_Led  &color1=1& hw_state=0 & (sound=0 | move=0)& Rec_motif1=false & staggered=false-> q_r_high_c1*beta1 : (s1'=finish) + 1-(q_r_high_c1*beta1) : (s1'=low_Led);// default (empirical)
  [step] s1=med_Led  &color1=1& hw_state=0 & (move=1)& Rec_motif1=false & staggered=false-> 1-(q_r_high_c1*beta1) : (s1'=task_failed) + q_r_high_c1*beta1 : (s1'=low_Led);// when movement un-intentionally

//low led white
  [step] s1=low_Led &color1=3& (hw_state=2 |env_state1=2) & Rec_motif1=false & staggered=false-> 1-(q_w_high_c1*beta2) : (s1'=finish) + q_w_high_c1*beta2 : (s1'=task_failed); 
  [step] s1=low_Led &color1=3& (hw_state=0 |hw_state=1)& (env_state1=1 |env_state1=0)& Rec_motif1=false & staggered=false-> q_w_high_c1*beta2 : (s1'=finish) + 1-(q_w_high_c1*beta2) : (s1'=task_failed);// default
//low led Blue
  [step] s1=low_Led &color1=2& (hw_state=2 |env_state1=2)& Rec_motif1=false & staggered=false -> 1-(q_b_high_c1*beta2) : (s1'=finish) + q_b_high_c1*beta2 : (s1'=task_failed); 
  [step] s1=low_Led &color1=2& (hw_state=0 |hw_state=1)& (env_state1=1 |env_state1=0)& Rec_motif1=false & staggered=false-> q_b_high_c1*beta2 : (s1'=finish) + 1-(q_b_high_c1*beta2) : (s1'=task_failed);// default
//low led Red
   [step] s1=low_Led &color1=1& (hw_state=2 |env_state1=2)& Rec_motif1=false & staggered=false -> 1-(q_r_high_c1*beta2) : (s1'=finish) + q_r_high_c1*beta2 : (s1'=task_failed); 
  [step] s1=low_Led &color1=1& (hw_state=0 |hw_state=1)& (env_state1=1 |env_state1=0)& Rec_motif1=false & staggered=false-> q_r_high_c1*beta2 : (s1'=finish) + 1-(q_r_high_c1*beta2) : (s1'=task_failed);// default
//////////////////////////////
  // Probabilistic transitions (BB1-specific)RECTANGLE pattern syncroninzed with other clusters
///////////////////////////////
 //high pitch
  [step] s1=high_pitch & hw_state=2 & Rec_motif1=true & staggered=false -> (s1'=low_pitch); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_pitch & hw_state=1 & Rec_motif1=true & staggered=false -> (s1'=med_pitch);// Priority 2: hardware MED -> reduce one level
  [step] s1=high_pitch & hw_state=0 & env_state1=2  & move=1& Rec_motif1=true & staggered=false ->(1-q_high_pitch_c1)/2 : (s1'=med_pitch) + 1-((1-q_high_pitch_c1)/2) : (s1'=low_pitch);// Priority 3: environment sensing
  [step] s1=high_pitch & hw_state=0 & env_state1= 0 & move=1& Rec_motif1=true & staggered=false->(1-q_high_pitch_c1) : (s1'=task_failed)+ q_high_pitch_c1/2 : (s1'=med_pitch) + q_high_pitch_c1/2 : (s1'=low_pitch);// when move (no direct intention of fallback), chance of failure occurs
  [step] s1=high_pitch & hw_state=0 & env_state1=1  & sound=1 & Rec_motif1=true & staggered=false->1-((1-q_high_pitch_c1)/2) : (s1'=med_pitch) + (1-q_high_pitch_c1)/2 : (s1'=low_pitch);
  [step] s1=high_pitch& hw_state=0   & (sound=0 & move=0) & Rec_motif1=true & staggered=false-> q_high_pitch_c1  : (s1'=finish) + (1-q_high_pitch_c1)/2 : (s1'=med_pitch) + (1-q_high_pitch_c1)/2 : (s1'=low_pitch); // default

//med pitch
  [step] s1=med_pitch & (hw_state=2 |hw_state=1) & Rec_motif1=true & staggered=false -> (s1'=low_pitch); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [step] s1=med_pitch & hw_state=0 & (env_state1=2 |env_state1=1)  &( sound=1 |move=1)& Rec_motif1=true & staggered=false -> (s1'=low_pitch);// Priority 3: environment sensing
  [step] s1=med_pitch & hw_state=0 & env_state1= 0 & move=1& Rec_motif1=true & staggered=false-> 1-(q_high_pitch_c1*beta1) : (s1'=task_failed)+ q_high_pitch_c1*beta1 : (s1'=low_pitch);// when move (no direct intention of fallback), chance of failure occurs
  [step] s1=med_pitch& hw_state=0  & (sound=0 & move=0)& Rec_motif1=true & staggered=false-> q_high_pitch_c1*beta1 : (s1'=finish) + (1-(q_high_pitch_c1*beta1)) : (s1'=low_pitch);// default

//low pitch
  [step] s1=low_pitch & (hw_state=2 |env_state1=2)& Rec_motif1=true & staggered=false -> (1-(q_high_pitch_c1*beta2)) : (s1'=finish) + q_high_pitch_c1*beta2 : (s1'=task_failed); 
  [step] s1=low_pitch & (hw_state=1 |env_state1=1)& Rec_motif1=true & staggered=false -> (1-((q_high_pitch_c1*beta2)/2)) : (s1'=finish) + (q_high_pitch_c1*beta2)/2 : (s1'=task_failed); 
  [step] s1=low_pitch & hw_state=0 & Rec_motif1=true & staggered=false-> q_high_pitch_c1*beta2 : (s1'=finish) + (1-(q_high_pitch_c1*beta2)) : (s1'=task_failed);//even if movement it is already very low consuming and it is the lowest state so failure exist but not as when a higher consuming state

//sensor-environment transition
  [step] s1=listen_on_moving_on & Rec_motif1=true & staggered=false-> 0.95 : (s1'=finish) + 0.05 : (s1'=task_failed);
  [step] s1=listen_on_moving_off & Rec_motif1=true & staggered=false-> 0.95 : (s1'=finish) + 0.05 : (s1'=task_failed);
  [step] s1=listen_off_moving_on& Rec_motif1=true & staggered=false -> 0.95 : (s1'=finish) + 0.05 : (s1'=task_failed);

//high-high simultaneous actuating white
  [step] s1=high_blink_high_buzz &color1=3 & hw_state=2 & Rec_motif1=true & staggered=false -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_blink_high_buzz &color1=3 & hw_state=1 & Rec_motif1=true & staggered=false -> 0.5 : (s1'=high_blink_med_buzz) +0.5: (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level
  [step] s1=high_blink_high_buzz &color1=3 & hw_state=0 & env_state1=2  & move=1& Rec_motif1=true & staggered=false->0.5 : (s1'=med_blink_high_buzz) + 0.5 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=high_blink_high_buzz &color1=3 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=true & staggered=false->1-((q_w_high_c1*theta)*q_high_pitch_c1) : (s1'=task_failed)+((q_w_high_c1*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + ((q_w_high_c1*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s1=high_blink_high_buzz &color1=3 & hw_state=0 & env_state1=1  & sound=1& Rec_motif1=true & staggered=false->(q_w_high_c1*theta)*q_high_pitch_c1 : (s1'=high_blink_med_buzz) + 1-((q_w_high_c1*theta)*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [step] s1=high_blink_high_buzz &color1=3 & hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=true & staggered=false -> (q_w_high_c1*theta)*q_high_pitch_c1: (s1'=finish) + (1-((q_w_high_c1*theta)*q_high_pitch_c1))/3 : (s1'=high_blink_med_buzz) + (1-((q_w_high_c1*theta)*q_high_pitch_c1))/3 : (s1'=med_blink_high_buzz) + (1-((q_w_high_c1*theta)*q_high_pitch_c1))/3 : (s1'=med_blink_med_buzz); //default(Empirical)
//high-high simultaneous actuating blue
  [step] s1=high_blink_high_buzz &color1=2 & hw_state=2 & Rec_motif1=true & staggered=false -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_blink_high_buzz &color1=2 & hw_state=1 & Rec_motif1=true & staggered=false -> 0.5 : (s1'=high_blink_med_buzz) +0.5: (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level
  [step] s1=high_blink_high_buzz &color1=2 & hw_state=0 & env_state1=2  & move=1& Rec_motif1=true & staggered=false->0.5 : (s1'=med_blink_high_buzz) + 0.5 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=high_blink_high_buzz &color1=2 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=true & staggered=false->1-((q_b_high_c1*theta)*q_high_pitch_c1) : (s1'=task_failed)+((q_b_high_c1*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + ((q_b_high_c1*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s1=high_blink_high_buzz &color1=2 & hw_state=0 & env_state1=1  & sound=1& Rec_motif1=true & staggered=false->(q_b_high_c1*theta)*q_high_pitch_c1 : (s1'=high_blink_med_buzz) + 1-((q_b_high_c1*theta)*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [step] s1=high_blink_high_buzz &color1=2 & hw_state=0 & env_state1=0  & (sound=0 & move=0) & Rec_motif1=true & staggered=false-> (q_b_high_c1*theta)*q_high_pitch_c1: (s1'=finish) + (1-((q_b_high_c1*theta)*q_high_pitch_c1))/3 : (s1'=high_blink_med_buzz) + (1-((q_b_high_c1*theta)*q_high_pitch_c1))/3 : (s1'=med_blink_high_buzz) + (1-((q_b_high_c1*theta)*q_high_pitch_c1))/3 : (s1'=med_blink_med_buzz); //default(Empirical)
//high-high simultaneous actuating RED
  [step] s1=high_blink_high_buzz &color1=1 & hw_state=2 & Rec_motif1=true & staggered=false -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_blink_high_buzz &color1=1 & hw_state=1 & Rec_motif1=true & staggered=false -> 0.5 : (s1'=high_blink_med_buzz) +0.5: (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level
  [step] s1=high_blink_high_buzz &color1=1 & hw_state=0 & env_state1=2  & move=1& Rec_motif1=true & staggered=false->0.5 : (s1'=med_blink_high_buzz) + 0.5 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=high_blink_high_buzz &color1=1 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=true & staggered=false->1-((q_r_high_c1*theta)*q_high_pitch_c1) : (s1'=task_failed)+((q_r_high_c1*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + ((q_r_high_c1*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s1=high_blink_high_buzz &color1=1 & hw_state=0 & env_state1=1  & sound=1& Rec_motif1=true & staggered=false->(q_r_high_c1*theta)*q_high_pitch_c1 : (s1'=high_blink_med_buzz) + 1-((q_r_high_c1*theta)*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [step] s1=high_blink_high_buzz &color1=1 & hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=true & staggered=false -> (q_r_high_c1*theta)*q_high_pitch_c1: (s1'=finish) + (1-((q_r_high_c1*theta)*q_high_pitch_c1))/3 : (s1'=high_blink_med_buzz) + (1-((q_r_high_c1*theta)*q_high_pitch_c1))/3 : (s1'=med_blink_high_buzz) + (1-((q_r_high_c1*theta)*q_high_pitch_c1))/3 : (s1'=med_blink_med_buzz); //default(Empirical)

//high-med simultaneous actuating white
  [step] s1=high_blink_med_buzz &color1=3 & hw_state=2& Rec_motif1=true & staggered=false  -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_blink_med_buzz &color1=3 & hw_state=1 & Rec_motif1=true & staggered=false -> 0.5 : (s1'=med_blink_med_buzz) + 0.5 : (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level /equal probabilities cause it deoends on user preferences
  [step] s1=high_blink_med_buzz &color1=3& hw_state=0 & env_state1=2  & move=1& Rec_motif1=true & staggered=false->1-((q_w_high_c1*theta)*q_high_pitch_c1) : (s1'=med_blink_high_buzz) + (q_w_high_c1*theta)*q_high_pitch_c1 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=high_blink_high_buzz &color1=3 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=true & staggered=false->1-((q_w_high_c1*theta)*q_high_pitch_c1) : (s1'=task_failed)+((q_w_high_c1*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + ((q_w_high_c1*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s1=high_blink_med_buzz &color1=3& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=true & staggered=false->(q_w_high_c1*theta)*q_high_pitch_c1  : (s1'=med_blink_med_buzz) + 1-((q_w_high_c1*theta)*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [step] s1=high_blink_med_buzz &color1=3& hw_state=0 & env_state1=0  & (sound=0 & move=0) & Rec_motif1=true & staggered=false-> (q_w_high_c1*theta)*q_high_pitch_c1 : (s1'=finish)  + (1-((q_w_high_c1*theta)*q_high_pitch_c1))/2 : (s1'=med_blink_high_buzz) + (1-((q_w_high_c1*theta)*q_high_pitch_c1))/2 : (s1'=med_blink_med_buzz); //default(empirical)
 //high-med simultaneous actuating blue
   [step] s1=high_blink_med_buzz &color1=2 & hw_state=2 & Rec_motif1=true & staggered=false -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_blink_med_buzz &color1=2 & hw_state=1 & Rec_motif1=true & staggered=false -> 0.5 : (s1'=med_blink_med_buzz) + 0.5 : (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level /equal probabilities cause it deoends on user preferences
  [step] s1=high_blink_med_buzz &color1=2& hw_state=0 & env_state1=2  & move=1& Rec_motif1=true & staggered=false->1-((q_b_high_c1*theta)*q_high_pitch_c1) : (s1'=med_blink_high_buzz) + (q_b_high_c1*theta)*q_high_pitch_c1 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=high_blink_high_buzz &color1=2 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=true & staggered=false->1-((q_b_high_c1*theta)*q_high_pitch_c1) : (s1'=task_failed)+((q_b_high_c1*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + ((q_b_high_c1*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s1=high_blink_med_buzz &color1=2& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=true & staggered=false->(q_b_high_c1*theta)*q_high_pitch_c1  : (s1'=med_blink_med_buzz) + 1-((q_b_high_c1*theta)*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [step] s1=high_blink_med_buzz &color1=2& hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=true & staggered=false -> (q_b_high_c1*theta)*q_high_pitch_c1 : (s1'=finish)  + (1-((q_b_high_c1*theta)*q_high_pitch_c1))/2 : (s1'=med_blink_high_buzz) + (1-((q_b_high_c1*theta)*q_high_pitch_c1))/2 : (s1'=med_blink_med_buzz); //default(empirical)
 
//high-med simultaneous actuating Red
  [step] s1=high_blink_med_buzz &color1=1 & hw_state=2 & Rec_motif1=true & staggered=false -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_blink_med_buzz &color1=1 & hw_state=1 & Rec_motif1=true & staggered=false -> 0.5 : (s1'=med_blink_med_buzz) + 0.5 : (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level /equal probabilities cause it deoends on user preferences
  [step] s1=high_blink_med_buzz &color1=1& hw_state=0 & env_state1=2  & move=1& Rec_motif1=true & staggered=false->1-((q_r_high_c1*theta)*q_high_pitch_c1) : (s1'=med_blink_high_buzz) + (q_r_high_c1*theta)*q_high_pitch_c1 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=high_blink_high_buzz &color1=1 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=true & staggered=false->1-((q_r_high_c1*theta)*q_high_pitch_c1) : (s1'=task_failed)+((q_r_high_c1*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + ((q_r_high_c1*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s1=high_blink_med_buzz &color1=1& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=true & staggered=false->(q_r_high_c1*theta)*q_high_pitch_c1  : (s1'=med_blink_med_buzz) + 1-((q_r_high_c1*theta)*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [step] s1=high_blink_med_buzz &color1=1& hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=true & staggered=false -> (q_r_high_c1*theta)*q_high_pitch_c1 : (s1'=finish)  + (1-((q_r_high_c1*theta)*q_high_pitch_c1))/2 : (s1'=med_blink_high_buzz) + (1-((q_r_high_c1*theta)*q_high_pitch_c1))/2 : (s1'=med_blink_med_buzz); //default(empirical)
 

 //med-high simultaneous actuating white
  [step] s1=med_blink_high_buzz &color1=3 & (hw_state=2 | hw_state=1) & Rec_motif1=true & staggered=false -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=med_blink_high_buzz &color1=3 & hw_state=0 & (env_state1=2 | env_state1=1)   & (sound=1 | move=1)& Rec_motif1=true & staggered=false-> (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=med_blink_high_buzz &color1=3 & hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=true & staggered=false-> (q_w_high_c1*theta)*q_high_pitch_c1*beta1 : (s1'=finish)  + 1-((q_w_high_c1*theta)*q_high_pitch_c1*beta1) : (s1'=med_blink_med_buzz); //default(empirical)
  [step] s1=med_blink_high_buzz &color1=3 & hw_state=0 & env_state1=0  & ( move=1)& Rec_motif1=true & staggered=false-> 1-((q_w_high_c1*theta)*q_high_pitch_c1*beta1) : (s1'=task_failed)  + (q_w_high_c1*theta)*q_high_pitch_c1*beta1 : (s1'=med_blink_med_buzz); //when movement un-intentionally
 //med-high simultaneous actuating blue
  [step] s1=med_blink_high_buzz &color1=2 & (hw_state=2 | hw_state=1)& Rec_motif1=true & staggered=false  -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=med_blink_high_buzz &color1=2 & hw_state=0 & (env_state1=2 | env_state1=1)   & (sound=1 | move=1)& Rec_motif1=true & staggered=false-> (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=med_blink_high_buzz &color1=2 & hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=true & staggered=false-> (q_b_high_c1*theta)*q_high_pitch_c1*beta1 : (s1'=finish)  + 1-((q_b_high_c1*theta)*q_high_pitch_c1*beta1) : (s1'=med_blink_med_buzz); //default(empirical)
  [step] s1=med_blink_high_buzz &color1=2 & hw_state=0 & env_state1=0  & ( move=1)& Rec_motif1=true & staggered=false-> 1-((q_b_high_c1*theta)*q_high_pitch_c1*beta1) : (s1'=task_failed)  + (q_b_high_c1*theta)*q_high_pitch_c1*beta1 : (s1'=med_blink_med_buzz); //when movement un-intentionally
 //med-high simultaneous actuating Red
  [step] s1=med_blink_high_buzz &color1=1 & (hw_state=2 | hw_state=1) & Rec_motif1=true & staggered=false -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=med_blink_high_buzz &color1=1 & hw_state=0 & (env_state1=2 | env_state1=1)   & (sound=1 | move=1)& Rec_motif1=true & staggered=false-> (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=med_blink_high_buzz &color1=1 & hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=true & staggered=false-> (q_r_high_c1*theta)*q_high_pitch_c1*beta1 : (s1'=finish)  + 1-((q_r_high_c1*theta)*q_high_pitch_c1*beta1) : (s1'=med_blink_med_buzz); //default(empirical)
  [step] s1=med_blink_high_buzz &color1=1 & hw_state=0 & env_state1=0  & ( move=1) & Rec_motif1=true & staggered=false-> 1-((q_r_high_c1*theta)*q_high_pitch_c1*beta1) : (s1'=task_failed)  + (q_r_high_c1*theta)*q_high_pitch_c1*beta1 : (s1'=med_blink_med_buzz); //when movement un-intentionally



//med-med simultaneous actuating white
  [step] s1=med_blink_med_buzz &color1=3& (hw_state=2 |env_state1=2)& Rec_motif1=true & staggered=false -> 1-((q_w_high_c1*theta)*q_high_pitch_c1*beta2) : (s1'=finish) + ((q_w_high_c1*theta)*q_high_pitch_c1*beta2) : (s1'=task_failed); 
  [step] s1=med_blink_med_buzz &color1=3& (hw_state=1 |env_state1=1) & Rec_motif1=true & staggered=false-> 0.5 : (s1'=finish) + 0.5 : (s1'=task_failed); 
  [step] s1=med_blink_med_buzz &color1=3& hw_state=0& Rec_motif1=true & staggered=false-> ((q_w_high_c1*theta)*q_high_pitch_c1*beta2) : (s1'=finish) + 1-((q_w_high_c1*theta)*q_high_pitch_c1*beta2) : (s1'=task_failed);//empirical data
//med-med simultaneous actuating Blue
[step] s1=med_blink_med_buzz &color1=2& (hw_state=2 |env_state1=2) & Rec_motif1=true & staggered=false-> 1-((q_b_high_c1*theta)*q_high_pitch_c1*beta2) : (s1'=finish) + ((q_b_high_c1*theta)*q_high_pitch_c1*beta2) : (s1'=task_failed); 
  [step] s1=med_blink_med_buzz &color1=2& (hw_state=1 |env_state1=1)& Rec_motif1=true & staggered=false -> 0.5 : (s1'=finish) + 0.5 : (s1'=task_failed); 
  [step] s1=med_blink_med_buzz &color1=2& hw_state=0& Rec_motif1=true & staggered=false-> ((q_b_high_c1*theta)*q_high_pitch_c1*beta2) : (s1'=finish) + 1-((q_b_high_c1*theta)*q_high_pitch_c1*beta2) : (s1'=task_failed);//empirical data
//med-med simultaneous actuating Red
  [step] s1=med_blink_med_buzz &color1=1& (hw_state=2 |env_state1=2)& Rec_motif1=true & staggered=false -> 1-((q_r_high_c1*theta)*q_high_pitch_c1*beta2) : (s1'=finish) + ((q_r_high_c1*theta)*q_high_pitch_c1*beta2) : (s1'=task_failed); 
  [step] s1=med_blink_med_buzz &color1=1& (hw_state=1 |env_state1=1) & Rec_motif1=true & staggered=false-> 0.5 : (s1'=finish) + 0.5 : (s1'=task_failed); 
  [step] s1=med_blink_med_buzz &color1=1& hw_state=0 & Rec_motif1=true & staggered=false-> ((q_r_high_c1*theta)*q_high_pitch_c1*beta2) : (s1'=finish) + 1-((q_r_high_c1*theta)*q_high_pitch_c1*beta2) : (s1'=task_failed);//empirical data

//High LED white
  [step] s1=high_Led  &color1=3& hw_state=2 & Rec_motif1=true & staggered=false -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_Led  &color1=3& hw_state=1 & Rec_motif1=true & staggered=false -> (s1'=med_Led);// Priority 2: hardware MED -> reduce one level
  [step] s1=high_Led  &color1=3& hw_state=0 & env_state1=2 & move=1& Rec_motif1=true & staggered=false->0.5  : (s1'=med_Led) + 0.5 : (s1'=low_Led);// Priority 3: environment sensing
  [step] s1=high_Led  &color1=3& hw_state=0 & env_state1=0 & move=1& Rec_motif1=true & staggered=false->(1-(q_w_high_c1*theta)) : (s1'=task_failed)  +(q_w_high_c1*theta)/2  : (s1'=med_Led) + (q_w_high_c1*theta)/2 : (s1'=low_Led);// when movement un-intentionally
  [step] s1=high_Led  &color1=3& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=true & staggered=false->0.5 : (s1'=med_Led) + 0.5 : (s1'=low_Led);
  [step] s1=high_Led  &color1=3& hw_state=0 & env_state1=0 & (sound=0 & move=0) & Rec_motif1=true & staggered=false-> (q_w_high_c1*theta) : (s1'=finish) + (1-(q_w_high_c1*theta))/2 : (s1'=med_Led) + (1-(q_w_high_c1*theta))/2 : (s1'=low_Led); // default (empirical)
//High LED Blue
  [step] s1=high_Led  &color1=2& hw_state=2 & Rec_motif1=true & staggered=false -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_Led  &color1=2& hw_state=1 & Rec_motif1=true & staggered=false -> (s1'=med_Led);// Priority 2: hardware MED -> reduce one level
  [step] s1=high_Led  &color1=2& hw_state=0 & env_state1=2 & move=1& Rec_motif1=true & staggered=false->0.5  : (s1'=med_Led) + 0.5 : (s1'=low_Led);// Priority 3: environment sensing
  [step] s1=high_Led  &color1=2& hw_state=0 & env_state1=0 & move=1& Rec_motif1=true & staggered=false->(1-(q_b_high_c1*theta)) : (s1'=task_failed)  +(q_b_high_c1*theta)/2  : (s1'=med_Led) + (q_b_high_c1*theta)/2 : (s1'=low_Led);// when movement un-intentionally
  [step] s1=high_Led  &color1=2& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=true & staggered=false->0.5 : (s1'=med_Led) + 0.5 : (s1'=low_Led);
  [step] s1=high_Led  &color1=2& hw_state=0 & env_state1=0 & (sound=0 & move=0)& Rec_motif1=true & staggered=false -> (q_b_high_c1*theta) : (s1'=finish) + (1-(q_b_high_c1*theta))/2 : (s1'=med_Led) + (1-(q_b_high_c1*theta))/2 : (s1'=low_Led); // default (empirical)
//High LED Red
   [step] s1=high_Led  &color1=1& hw_state=2 & Rec_motif1=true & staggered=false -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_Led  &color1=1& hw_state=1 & Rec_motif1=true & staggered=false -> (s1'=med_Led);// Priority 2: hardware MED -> reduce one level
  [step] s1=high_Led  &color1=1& hw_state=0 & env_state1=2 & move=1& Rec_motif1=true & staggered=false->0.5  : (s1'=med_Led) + 0.5 : (s1'=low_Led);// Priority 3: environment sensing
  [step] s1=high_Led  &color1=1& hw_state=0 & env_state1=0 & move=1& Rec_motif1=true & staggered=false->(1-(q_r_high_c1*theta)) : (s1'=task_failed)  +(q_r_high_c1*theta)/2  : (s1'=med_Led) + (q_r_high_c1*theta)/2 : (s1'=low_Led);// when movement un-intentionally
  [step] s1=high_Led  &color1=1& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=true & staggered=false->0.5 : (s1'=med_Led) + 0.5 : (s1'=low_Led);
  [step] s1=high_Led  &color1=1& hw_state=0 & env_state1=0 & (sound=0 & move=0)& Rec_motif1=true & staggered=false -> (q_r_high_c1*theta) : (s1'=finish) + (1-(q_r_high_c1*theta))/2 : (s1'=med_Led) + (1-(q_r_high_c1*theta))/2 : (s1'=low_Led); // default (empirical)

//Med LED white
  [step] s1=med_Led  &color1=3& (hw_state=2 |hw_state=1) & Rec_motif1=true & staggered=false -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [step] s1=med_Led  &color1=3& hw_state=0 & (env_state1=2 |env_state1=1)  & (sound=1 | move=1)& Rec_motif1=true & staggered=false-> (s1'=low_Led);// Priority 3: environment sensing
  [step] s1=med_Led  &color1=3& hw_state=0 & (sound=0 | move=0)& Rec_motif1=true & staggered=false-> (q_w_high_c1*theta)*beta1 : (s1'=finish) + 1-((q_w_high_c1*theta)*beta1) : (s1'=low_Led);// default (empirical)
  [step] s1=med_Led  &color1=3& hw_state=0 & (move=1)& Rec_motif1=true & staggered=false-> 1-((q_w_high_c1*theta)*beta1) : (s1'=task_failed) + (q_w_high_c1*theta)*beta1 : (s1'=low_Led);// when movement un-intentionally
//Med LED Blue
  [step] s1=med_Led   &color1=2& (hw_state=2 |hw_state=1) & Rec_motif1=true & staggered=false -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [step] s1=med_Led  &color1=2& hw_state=0 & (env_state1=2 |env_state1=1)  & (sound=1 | move=1)& Rec_motif1=true & staggered=false-> (s1'=low_Led);// Priority 3: environment sensing
  [step] s1=med_Led  &color1=2& hw_state=0 & (sound=0 | move=0)& Rec_motif1=true & staggered=false-> (q_b_high_c1*theta)*beta1 : (s1'=finish) + 1-((q_b_high_c1*theta)*beta1) : (s1'=low_Led);// default (empirical)
  [step] s1=med_Led  &color1=2& hw_state=0 & (move=1)& Rec_motif1=true & staggered=false-> 1-((q_b_high_c1*theta)*beta1) : (s1'=task_failed) + (q_b_high_c1*theta)*beta1 : (s1'=low_Led);// when movement un-intentionally
//Med LED Red
  [step] s1=med_Led   &color1=1& (hw_state=2 |hw_state=1) & Rec_motif1=true & staggered=false -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [step] s1=med_Led  &color1=1& hw_state=0 & (env_state1=2 |env_state1=1)  & (sound=1 | move=1)& Rec_motif1=true & staggered=false-> (s1'=low_Led);// Priority 3: environment sensing
  [step] s1=med_Led  &color1=1& hw_state=0 & (sound=0 | move=0)& Rec_motif1=true & staggered=false-> (q_r_high_c1*theta)*beta1 : (s1'=finish) + 1-((q_r_high_c1*theta)*beta1) : (s1'=low_Led);// default (empirical)
  [step] s1=med_Led  &color1=1& hw_state=0 & (move=1)& Rec_motif1=true & staggered=false-> 1-((q_r_high_c1*theta)*beta1) : (s1'=task_failed) + (q_r_high_c1*theta)*beta1 : (s1'=low_Led);// when movement un-intentionally

//low led white
  [step] s1=low_Led &color1=3& (hw_state=2 |env_state1=2) & Rec_motif1=true & staggered=false-> 1-((q_w_high_c1*theta)*beta2) : (s1'=finish) + (q_w_high_c1*theta)*beta2 : (s1'=task_failed); 
  [step] s1=low_Led &color1=3& (hw_state=0 |hw_state=1)& (env_state1=1 |env_state1=0)& Rec_motif1=true & staggered=false-> (q_w_high_c1*theta)*beta2 : (s1'=finish) + 1-((q_w_high_c1*theta)*beta2) : (s1'=task_failed);// default
//low led Blue
  [step] s1=low_Led &color1=2& (hw_state=2 |env_state1=2)& Rec_motif1=true & staggered=false -> 1-((q_b_high_c1*theta)*beta2) : (s1'=finish) + (q_b_high_c1*theta)*beta2 : (s1'=task_failed); 
  [step] s1=low_Led &color1=2& (hw_state=0 |hw_state=1)& (env_state1=1 |env_state1=0)& Rec_motif1=true & staggered=false-> (q_b_high_c1*theta)*beta2 : (s1'=finish) + 1-((q_b_high_c1*theta)*beta2) : (s1'=task_failed);// default
//low led Red
   [step] s1=low_Led &color1=1& (hw_state=2 |env_state1=2)& Rec_motif1=true & staggered=false -> 1-((q_r_high_c1*theta)*beta2) : (s1'=finish) + (q_r_high_c1*theta)*beta2 : (s1'=task_failed); 
  [step] s1=low_Led &color1=1& (hw_state=0 |hw_state=1)& (env_state1=1 |env_state1=0)& Rec_motif1=true & staggered=false-> (q_r_high_c1*theta)*beta2 : (s1'=finish) + 1-((q_r_high_c1*theta)*beta2) : (s1'=task_failed);// default
//////////////////////////////

//FINISH
 [step] s1=finish -> (s1'=finish);// absorbing states if synced with other blocks
 [step] s1=task_failed -> (s1'=task_failed); // absorbing states if synced with other blocks
  // Reset on tick
  [Task_increment] s1=finish  -> (s1'=start_task) ;
  [Task_increment] s1=task_failed  -> (s1'=start_task) ;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 // Probabilistic transitions (BB1-specific) LINE pattern staggered with other clusters
///////////////////////////////////
//high pitch
  [stag1] s1=high_pitch & hw_state=2 & Rec_motif1=false & staggered=true -> (s1'=low_pitch); // Priority 1: hardware HIGH -> immediate downgrade
  [stag1] s1=high_pitch & hw_state=1 & Rec_motif1=false & staggered=true -> (s1'=med_pitch);// Priority 2: hardware MED -> reduce one level
  [stag1] s1=high_pitch & hw_state=0 & env_state1=2  & move=1& Rec_motif1=false & staggered=true ->(1-q_high_pitch_c1)/2 : (s1'=med_pitch) + 1-((1-q_high_pitch_c1)/2) : (s1'=low_pitch);// Priority 3: environment sensing
  [stag1] s1=high_pitch & hw_state=0 & env_state1= 0 & move=1& Rec_motif1=false & staggered=true->(1-q_high_pitch_c1) : (s1'=task_failed)+ q_high_pitch_c1/2 : (s1'=med_pitch) + q_high_pitch_c1/2 : (s1'=low_pitch);// when move (no direct intention of fallback), chance of failure occurs
  [stag1] s1=high_pitch & hw_state=0 & env_state1=1  & sound=1 & Rec_motif1=false & staggered=true->1-((1-q_high_pitch_c1)/2) : (s1'=med_pitch) + (1-q_high_pitch_c1)/2 : (s1'=low_pitch);
  [stag1] s1=high_pitch& hw_state=0   & (sound=0 & move=0) & Rec_motif1=false & staggered=true-> q_high_pitch_c1  : (s1'=finish) + (1-q_high_pitch_c1)/2 : (s1'=med_pitch) + (1-q_high_pitch_c1)/2 : (s1'=low_pitch); // default

//med pitch
  [stag1] s1=med_pitch & (hw_state=2 |hw_state=1) & Rec_motif1=false & staggered=true -> (s1'=low_pitch); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [stag1] s1=med_pitch & hw_state=0 & (env_state1=2 |env_state1=1)  &( sound=1 |move=1)& Rec_motif1=false & staggered=true -> (s1'=low_pitch);// Priority 3: environment sensing
  [stag1] s1=med_pitch & hw_state=0 & env_state1= 0 & move=1& Rec_motif1=false & staggered=true-> 1-(q_high_pitch_c1*beta1) : (s1'=task_failed)+ q_high_pitch_c1*beta1 : (s1'=low_pitch);// when move (no direct intention of fallback), chance of failure occurs
  [stag1] s1=med_pitch& hw_state=0  & (sound=0 & move=0)& Rec_motif1=false & staggered=true-> q_high_pitch_c1*beta1 : (s1'=finish) + (1-(q_high_pitch_c1*beta1)) : (s1'=low_pitch);// default

//low pitch
  [stag1] s1=low_pitch & (hw_state=2 |env_state1=2)& Rec_motif1=false & staggered=true -> (1-(q_high_pitch_c1*beta2)) : (s1'=finish) + q_high_pitch_c1*beta2 : (s1'=task_failed); 
  [stag1] s1=low_pitch & (hw_state=1 |env_state1=1)& Rec_motif1=false & staggered=true -> (1-((q_high_pitch_c1*beta2)/2)) : (s1'=finish) + (q_high_pitch_c1*beta2)/2 : (s1'=task_failed); 
  [stag1] s1=low_pitch & hw_state=0 & Rec_motif1=false & staggered=true-> q_high_pitch_c1*beta2 : (s1'=finish) + (1-(q_high_pitch_c1*beta2)) : (s1'=task_failed);//even if movement it is already very low consuming and it is the lowest state so failure exist but not as when a higher consuming state

//sensor-environment transition
  [stag1] s1=listen_on_moving_on & Rec_motif1=false & staggered=true-> 0.95 : (s1'=finish) + 0.05 : (s1'=task_failed);
  [stag1] s1=listen_on_moving_off & Rec_motif1=false & staggered=true-> 0.95 : (s1'=finish) + 0.05 : (s1'=task_failed);
  [stag1] s1=listen_off_moving_on& Rec_motif1=false & staggered=true -> 0.95 : (s1'=finish) + 0.05 : (s1'=task_failed);

//high-high simultaneous actuating white
  [stag1] s1=high_blink_high_buzz &color1=3 & hw_state=2 & Rec_motif1=false & staggered=true -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag1] s1=high_blink_high_buzz &color1=3 & hw_state=1 & Rec_motif1=false & staggered=true -> 0.5 : (s1'=high_blink_med_buzz) +0.5: (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level
  [stag1] s1=high_blink_high_buzz &color1=3 & hw_state=0 & env_state1=2  & move=1& Rec_motif1=false & staggered=true->0.5 : (s1'=med_blink_high_buzz) + 0.5 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag1] s1=high_blink_high_buzz &color1=3 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=false & staggered=true->1-((q_w_high_c1*gamma)*q_high_pitch_c1) : (s1'=task_failed)+((q_w_high_c1*gamma)*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + ((q_w_high_c1*gamma)*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [stag1] s1=high_blink_high_buzz &color1=3 & hw_state=0 & env_state1=1  & sound=1& Rec_motif1=false & staggered=true->(q_w_high_c1*gamma)*q_high_pitch_c1 : (s1'=high_blink_med_buzz) + 1-((q_w_high_c1*gamma)*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [stag1] s1=high_blink_high_buzz &color1=3 & hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=false & staggered=true -> (q_w_high_c1*gamma)*q_high_pitch_c1: (s1'=finish) + (1-((q_w_high_c1*gamma)*q_high_pitch_c1))/3 : (s1'=high_blink_med_buzz) + (1-((q_w_high_c1*gamma)*q_high_pitch_c1))/3 : (s1'=med_blink_high_buzz) + (1-((q_w_high_c1*gamma)*q_high_pitch_c1))/3 : (s1'=med_blink_med_buzz); //default(Empirical)
//high-high simultaneous actuating blue
  [stag1] s1=high_blink_high_buzz &color1=2 & hw_state=2 & Rec_motif1=false & staggered=true -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag1] s1=high_blink_high_buzz &color1=2 & hw_state=1 & Rec_motif1=false & staggered=true -> 0.5 : (s1'=high_blink_med_buzz) +0.5: (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level
  [stag1] s1=high_blink_high_buzz &color1=2 & hw_state=0 & env_state1=2  & move=1& Rec_motif1=false & staggered=true->0.5 : (s1'=med_blink_high_buzz) + 0.5 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag1] s1=high_blink_high_buzz &color1=2 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=false & staggered=true->1-((q_b_high_c1*gamma)*q_high_pitch_c1) : (s1'=task_failed)+((q_b_high_c1*gamma)*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + ((q_b_high_c1*gamma)*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [stag1] s1=high_blink_high_buzz &color1=2 & hw_state=0 & env_state1=1  & sound=1& Rec_motif1=false & staggered=true->(q_b_high_c1*gamma)*q_high_pitch_c1 : (s1'=high_blink_med_buzz) + 1-((q_b_high_c1*gamma)*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [stag1] s1=high_blink_high_buzz &color1=2 & hw_state=0 & env_state1=0  & (sound=0 & move=0) & Rec_motif1=false & staggered=true-> (q_b_high_c1*gamma)*q_high_pitch_c1: (s1'=finish) + (1-((q_b_high_c1*gamma)*q_high_pitch_c1))/3 : (s1'=high_blink_med_buzz) + (1-((q_b_high_c1*gamma)*q_high_pitch_c1))/3 : (s1'=med_blink_high_buzz) + (1-((q_b_high_c1*gamma)*q_high_pitch_c1))/3 : (s1'=med_blink_med_buzz); //default(Empirical)
//high-high simultaneous actuating RED
  [stag1] s1=high_blink_high_buzz &color1=1 & hw_state=2 & Rec_motif1=false & staggered=true -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag1] s1=high_blink_high_buzz &color1=1 & hw_state=1 & Rec_motif1=false & staggered=true -> 0.5 : (s1'=high_blink_med_buzz) +0.5: (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level
  [stag1] s1=high_blink_high_buzz &color1=1 & hw_state=0 & env_state1=2  & move=1& Rec_motif1=false & staggered=true->0.5 : (s1'=med_blink_high_buzz) + 0.5 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag1] s1=high_blink_high_buzz &color1=1 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=false & staggered=true->1-((q_r_high_c1*gamma)*q_high_pitch_c1) : (s1'=task_failed)+((q_r_high_c1*gamma)*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + ((q_r_high_c1*gamma)*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [stag1] s1=high_blink_high_buzz &color1=1 & hw_state=0 & env_state1=1  & sound=1& Rec_motif1=false & staggered=true->(q_r_high_c1*gamma)*q_high_pitch_c1 : (s1'=high_blink_med_buzz) + 1-((q_r_high_c1*gamma)*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [stag1] s1=high_blink_high_buzz &color1=1 & hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=false & staggered=true -> (q_r_high_c1*gamma)*q_high_pitch_c1: (s1'=finish) + (1-((q_r_high_c1*gamma)*q_high_pitch_c1))/3 : (s1'=high_blink_med_buzz) + (1-((q_r_high_c1*gamma)*q_high_pitch_c1))/3 : (s1'=med_blink_high_buzz) + (1-((q_r_high_c1*gamma)*q_high_pitch_c1))/3 : (s1'=med_blink_med_buzz); //default(Empirical)

//high-med simultaneous actuating white
  [stag1] s1=high_blink_med_buzz &color1=3 & hw_state=2& Rec_motif1=false & staggered=true  -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag1] s1=high_blink_med_buzz &color1=3 & hw_state=1 & Rec_motif1=false & staggered=true -> 0.5 : (s1'=med_blink_med_buzz) + 0.5 : (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level /equal probabilities cause it deoends on user preferences
  [stag1] s1=high_blink_med_buzz &color1=3& hw_state=0 & env_state1=2  & move=1& Rec_motif1=false & staggered=true->1-((q_w_high_c1*gamma)*q_high_pitch_c1) : (s1'=med_blink_high_buzz) + (q_w_high_c1*gamma)*q_high_pitch_c1 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag1] s1=high_blink_high_buzz &color1=3 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=false & staggered=true->1-((q_w_high_c1*gamma)*q_high_pitch_c1) : (s1'=task_failed)+((q_w_high_c1*gamma)*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + ((q_w_high_c1*gamma)*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [stag1] s1=high_blink_med_buzz &color1=3& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=false & staggered=true->(q_w_high_c1*gamma)*q_high_pitch_c1  : (s1'=med_blink_med_buzz) + 1-((q_w_high_c1*gamma)*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [stag1] s1=high_blink_med_buzz &color1=3& hw_state=0 & env_state1=0  & (sound=0 & move=0) & Rec_motif1=false & staggered=true-> (q_w_high_c1*gamma)*q_high_pitch_c1 : (s1'=finish)  + (1-((q_w_high_c1*gamma)*q_high_pitch_c1))/2 : (s1'=med_blink_high_buzz) + (1-((q_w_high_c1*gamma)*q_high_pitch_c1))/2 : (s1'=med_blink_med_buzz); //default(empirical)
 //high-med simultaneous actuating blue
   [stag1] s1=high_blink_med_buzz &color1=2 & hw_state=2 & Rec_motif1=false & staggered=true -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag1] s1=high_blink_med_buzz &color1=2 & hw_state=1 & Rec_motif1=false & staggered=true -> 0.5 : (s1'=med_blink_med_buzz) + 0.5 : (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level /equal probabilities cause it deoends on user preferences
  [stag1] s1=high_blink_med_buzz &color1=2& hw_state=0 & env_state1=2  & move=1& Rec_motif1=false & staggered=true->1-((q_b_high_c1*gamma)*q_high_pitch_c1) : (s1'=med_blink_high_buzz) + (q_b_high_c1*gamma)*q_high_pitch_c1 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag1] s1=high_blink_high_buzz &color1=2 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=false & staggered=true->1-((q_b_high_c1*gamma)*q_high_pitch_c1) : (s1'=task_failed)+((q_b_high_c1*gamma)*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + ((q_b_high_c1*gamma)*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [stag1] s1=high_blink_med_buzz &color1=2& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=false & staggered=true->(q_b_high_c1*gamma)*q_high_pitch_c1  : (s1'=med_blink_med_buzz) + 1-((q_b_high_c1*gamma)*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [stag1] s1=high_blink_med_buzz &color1=2& hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=false & staggered=true -> (q_b_high_c1*gamma)*q_high_pitch_c1 : (s1'=finish)  + (1-((q_b_high_c1*gamma)*q_high_pitch_c1))/2 : (s1'=med_blink_high_buzz) + (1-((q_b_high_c1*gamma)*q_high_pitch_c1))/2 : (s1'=med_blink_med_buzz); //default(empirical)
 
//high-med simultaneous actuating Red
  [stag1] s1=high_blink_med_buzz &color1=1 & hw_state=2 & Rec_motif1=false & staggered=true -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag1] s1=high_blink_med_buzz &color1=1 & hw_state=1 & Rec_motif1=false & staggered=true -> 0.5 : (s1'=med_blink_med_buzz) + 0.5 : (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level /equal probabilities cause it deoends on user preferences
  [stag1] s1=high_blink_med_buzz &color1=1& hw_state=0 & env_state1=2  & move=1& Rec_motif1=false & staggered=true->1-((q_r_high_c1*gamma)*q_high_pitch_c1) : (s1'=med_blink_high_buzz) + (q_r_high_c1*gamma)*q_high_pitch_c1 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag1] s1=high_blink_high_buzz &color1=1 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=false & staggered=true->1-((q_r_high_c1*gamma)*q_high_pitch_c1) : (s1'=task_failed)+((q_r_high_c1*gamma)*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + ((q_r_high_c1*gamma)*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [stag1] s1=high_blink_med_buzz &color1=1& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=false & staggered=true->(q_r_high_c1*gamma)*q_high_pitch_c1  : (s1'=med_blink_med_buzz) + 1-((q_r_high_c1*gamma)*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [stag1] s1=high_blink_med_buzz &color1=1& hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=false & staggered=true -> (q_r_high_c1*gamma)*q_high_pitch_c1 : (s1'=finish)  + (1-((q_r_high_c1*gamma)*q_high_pitch_c1))/2 : (s1'=med_blink_high_buzz) + (1-((q_r_high_c1*gamma)*q_high_pitch_c1))/2 : (s1'=med_blink_med_buzz); //default(empirical)
 

 //med-high simultaneous actuating white
  [stag1] s1=med_blink_high_buzz &color1=3 & (hw_state=2 | hw_state=1) & Rec_motif1=false & staggered=true -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag1] s1=med_blink_high_buzz &color1=3 & hw_state=0 & (env_state1=2 | env_state1=1)   & (sound=1 | move=1)& Rec_motif1=false & staggered=true-> (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag1] s1=med_blink_high_buzz &color1=3 & hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=false & staggered=true-> (q_w_high_c1*gamma)*q_high_pitch_c1*beta1 : (s1'=finish)  + 1-((q_w_high_c1*gamma)*q_high_pitch_c1*beta1) : (s1'=med_blink_med_buzz); //default(empirical)
  [stag1] s1=med_blink_high_buzz &color1=3 & hw_state=0 & env_state1=0  & ( move=1)& Rec_motif1=false & staggered=true-> 1-((q_w_high_c1*gamma)*q_high_pitch_c1*beta1) : (s1'=task_failed)  + (q_w_high_c1*gamma)*q_high_pitch_c1*beta1 : (s1'=med_blink_med_buzz); //when movement un-intentionally
 //med-high simultaneous actuating blue
  [stag1] s1=med_blink_high_buzz &color1=2 & (hw_state=2 | hw_state=1)& Rec_motif1=false & staggered=true  -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag1] s1=med_blink_high_buzz &color1=2 & hw_state=0 & (env_state1=2 | env_state1=1)   & (sound=1 | move=1)& Rec_motif1=false & staggered=true-> (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag1] s1=med_blink_high_buzz &color1=2 & hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=false & staggered=true-> (q_b_high_c1*gamma)*q_high_pitch_c1*beta1 : (s1'=finish)  + 1-((q_b_high_c1*gamma)*q_high_pitch_c1*beta1) : (s1'=med_blink_med_buzz); //default(empirical)
  [stag1] s1=med_blink_high_buzz &color1=2 & hw_state=0 & env_state1=0  & ( move=1)& Rec_motif1=false & staggered=true-> 1-((q_b_high_c1*gamma)*q_high_pitch_c1*beta1) : (s1'=task_failed)  + (q_b_high_c1*gamma)*q_high_pitch_c1*beta1 : (s1'=med_blink_med_buzz); //when movement un-intentionally
 //med-high simultaneous actuating Red
  [stag1] s1=med_blink_high_buzz &color1=1 & (hw_state=2 | hw_state=1) & Rec_motif1=false & staggered=true -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag1] s1=med_blink_high_buzz &color1=1 & hw_state=0 & (env_state1=2 | env_state1=1)   & (sound=1 | move=1)& Rec_motif1=false & staggered=true-> (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag1] s1=med_blink_high_buzz &color1=1 & hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=false & staggered=true-> (q_r_high_c1*gamma)*q_high_pitch_c1*beta1 : (s1'=finish)  + 1-((q_r_high_c1*gamma)*q_high_pitch_c1*beta1) : (s1'=med_blink_med_buzz); //default(empirical)
  [stag1] s1=med_blink_high_buzz &color1=1 & hw_state=0 & env_state1=0  & ( move=1) & Rec_motif1=false & staggered=true-> 1-((q_r_high_c1*gamma)*q_high_pitch_c1*beta1) : (s1'=task_failed)  + (q_r_high_c1*gamma)*q_high_pitch_c1*beta1 : (s1'=med_blink_med_buzz); //when movement un-intentionally



//med-med simultaneous actuating white
  [stag1] s1=med_blink_med_buzz &color1=3& (hw_state=2 |env_state1=2)& Rec_motif1=false & staggered=true -> 1-((q_w_high_c1*gamma)*q_high_pitch_c1*beta2) : (s1'=finish) + ((q_w_high_c1*gamma)*q_high_pitch_c1*beta2) : (s1'=task_failed); 
  [stag1] s1=med_blink_med_buzz &color1=3& (hw_state=1 |env_state1=1) & Rec_motif1=false & staggered=true-> 0.5 : (s1'=finish) + 0.5 : (s1'=task_failed); 
  [stag1] s1=med_blink_med_buzz &color1=3& hw_state=0& Rec_motif1=false & staggered=true-> ((q_w_high_c1*gamma)*q_high_pitch_c1*beta2) : (s1'=finish) + 1-((q_w_high_c1*gamma)*q_high_pitch_c1*beta2) : (s1'=task_failed);//empirical data
//med-med simultaneous actuating Blue
[stag1] s1=med_blink_med_buzz &color1=2& (hw_state=2 |env_state1=2) & Rec_motif1=false & staggered=true-> 1-((q_b_high_c1*gamma)*q_high_pitch_c1*beta2) : (s1'=finish) + ((q_b_high_c1*gamma)*q_high_pitch_c1*beta2) : (s1'=task_failed); 
  [stag1] s1=med_blink_med_buzz &color1=2& (hw_state=1 |env_state1=1)& Rec_motif1=false & staggered=true -> 0.5 : (s1'=finish) + 0.5 : (s1'=task_failed); 
  [stag1] s1=med_blink_med_buzz &color1=2& hw_state=0& Rec_motif1=false & staggered=true-> ((q_b_high_c1*gamma)*q_high_pitch_c1*beta2) : (s1'=finish) + 1-((q_b_high_c1*gamma)*q_high_pitch_c1*beta2) : (s1'=task_failed);//empirical data
//med-med simultaneous actuating Red
  [stag1] s1=med_blink_med_buzz &color1=1& (hw_state=2 |env_state1=2)& Rec_motif1=false & staggered=true -> 1-((q_r_high_c1*gamma)*q_high_pitch_c1*beta2) : (s1'=finish) + ((q_r_high_c1*gamma)*q_high_pitch_c1*beta2) : (s1'=task_failed); 
  [stag1] s1=med_blink_med_buzz &color1=1& (hw_state=1 |env_state1=1) & Rec_motif1=false & staggered=true-> 0.5 : (s1'=finish) + 0.5 : (s1'=task_failed); 
  [stag1] s1=med_blink_med_buzz &color1=1& hw_state=0 & Rec_motif1=false & staggered=true-> ((q_r_high_c1*gamma)*q_high_pitch_c1*beta2) : (s1'=finish) + 1-((q_r_high_c1*gamma)*q_high_pitch_c1*beta2) : (s1'=task_failed);//empirical data

//High LED white
  [stag1] s1=high_Led  &color1=3& hw_state=2 & Rec_motif1=false & staggered=true -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade
  [stag1] s1=high_Led  &color1=3& hw_state=1 & Rec_motif1=false & staggered=true -> (s1'=med_Led);// Priority 2: hardware MED -> reduce one level
  [stag1] s1=high_Led  &color1=3& hw_state=0 & env_state1=2 & move=1& Rec_motif1=false & staggered=true->0.5  : (s1'=med_Led) + 0.5 : (s1'=low_Led);// Priority 3: environment sensing
  [stag1] s1=high_Led  &color1=3& hw_state=0 & env_state1=0 & move=1& Rec_motif1=false & staggered=true->(1-(q_w_high_c1*gamma)) : (s1'=task_failed)  +(q_w_high_c1*gamma)/2  : (s1'=med_Led) + (q_w_high_c1*gamma)/2 : (s1'=low_Led);// when movement un-intentionally
  [stag1] s1=high_Led  &color1=3& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=false & staggered=true->0.5 : (s1'=med_Led) + 0.5 : (s1'=low_Led);
  [stag1] s1=high_Led  &color1=3& hw_state=0 & env_state1=0 & (sound=0 & move=0) & Rec_motif1=false & staggered=true-> (q_w_high_c1*gamma) : (s1'=finish) + (1-(q_w_high_c1*gamma))/2 : (s1'=med_Led) + (1-(q_w_high_c1*gamma))/2 : (s1'=low_Led); // default (empirical)
//High LED Blue
  [stag1] s1=high_Led  &color1=2& hw_state=2 & Rec_motif1=false & staggered=true -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade
  [stag1] s1=high_Led  &color1=2& hw_state=1 & Rec_motif1=false & staggered=true -> (s1'=med_Led);// Priority 2: hardware MED -> reduce one level
  [stag1] s1=high_Led  &color1=2& hw_state=0 & env_state1=2 & move=1& Rec_motif1=false & staggered=true->0.5  : (s1'=med_Led) + 0.5 : (s1'=low_Led);// Priority 3: environment sensing
  [stag1] s1=high_Led  &color1=2& hw_state=0 & env_state1=0 & move=1& Rec_motif1=false & staggered=true->(1-(q_b_high_c1*gamma)) : (s1'=task_failed)  +(q_b_high_c1*gamma)/2  : (s1'=med_Led) + (q_b_high_c1*gamma)/2 : (s1'=low_Led);// when movement un-intentionally
  [stag1] s1=high_Led  &color1=2& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=false & staggered=true->0.5 : (s1'=med_Led) + 0.5 : (s1'=low_Led);
  [stag1] s1=high_Led  &color1=2& hw_state=0 & env_state1=0 & (sound=0 & move=0)& Rec_motif1=false & staggered=true -> (q_b_high_c1*gamma) : (s1'=finish) + (1-(q_b_high_c1*gamma))/2 : (s1'=med_Led) + (1-(q_b_high_c1*gamma))/2 : (s1'=low_Led); // default (empirical)
//High LED Red
   [stag1] s1=high_Led  &color1=1& hw_state=2 & Rec_motif1=false & staggered=true -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade
  [stag1] s1=high_Led  &color1=1& hw_state=1 & Rec_motif1=false & staggered=true -> (s1'=med_Led);// Priority 2: hardware MED -> reduce one level
  [stag1] s1=high_Led  &color1=1& hw_state=0 & env_state1=2 & move=1& Rec_motif1=false & staggered=true->0.5  : (s1'=med_Led) + 0.5 : (s1'=low_Led);// Priority 3: environment sensing
  [stag1] s1=high_Led  &color1=1& hw_state=0 & env_state1=0 & move=1& Rec_motif1=false & staggered=true->(1-(q_r_high_c1*gamma)) : (s1'=task_failed)  +(q_r_high_c1*gamma)/2  : (s1'=med_Led) + (q_r_high_c1*gamma)/2 : (s1'=low_Led);// when movement un-intentionally
  [stag1] s1=high_Led  &color1=1& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=false & staggered=true->0.5 : (s1'=med_Led) + 0.5 : (s1'=low_Led);
  [stag1] s1=high_Led  &color1=1& hw_state=0 & env_state1=0 & (sound=0 & move=0)& Rec_motif1=false & staggered=true -> (q_r_high_c1*gamma) : (s1'=finish) + (1-(q_r_high_c1*gamma))/2 : (s1'=med_Led) + (1-(q_r_high_c1*gamma))/2 : (s1'=low_Led); // default (empirical)

//Med LED white
  [stag1] s1=med_Led  &color1=3& (hw_state=2 |hw_state=1) & Rec_motif1=false & staggered=true -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [stag1] s1=med_Led  &color1=3& hw_state=0 & (env_state1=2 |env_state1=1)  & (sound=1 | move=1)& Rec_motif1=false & staggered=true-> (s1'=low_Led);// Priority 3: environment sensing
  [stag1] s1=med_Led  &color1=3& hw_state=0 & (sound=0 | move=0)& Rec_motif1=false & staggered=true-> (q_w_high_c1*gamma)*beta1 : (s1'=finish) + 1-((q_w_high_c1*gamma)*beta1) : (s1'=low_Led);// default (empirical)
  [stag1] s1=med_Led  &color1=3& hw_state=0 & (move=1)& Rec_motif1=false & staggered=true-> 1-((q_w_high_c1*gamma)*beta1) : (s1'=task_failed) + (q_w_high_c1*gamma)*beta1 : (s1'=low_Led);// when movement un-intentionally
//Med LED Blue
  [stag1] s1=med_Led   &color1=2& (hw_state=2 |hw_state=1) & Rec_motif1=false & staggered=true -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [stag1] s1=med_Led  &color1=2& hw_state=0 & (env_state1=2 |env_state1=1)  & (sound=1 | move=1)& Rec_motif1=false & staggered=true-> (s1'=low_Led);// Priority 3: environment sensing
  [stag1] s1=med_Led  &color1=2& hw_state=0 & (sound=0 | move=0)& Rec_motif1=false & staggered=true-> (q_b_high_c1*gamma)*beta1 : (s1'=finish) + 1-((q_b_high_c1*gamma)*beta1) : (s1'=low_Led);// default (empirical)
  [stag1] s1=med_Led  &color1=2& hw_state=0 & (move=1)& Rec_motif1=false & staggered=true-> 1-((q_b_high_c1*gamma)*beta1) : (s1'=task_failed) + (q_b_high_c1*gamma)*beta1 : (s1'=low_Led);// when movement un-intentionally
//Med LED Red
  [stag1] s1=med_Led   &color1=1& (hw_state=2 |hw_state=1) & Rec_motif1=false & staggered=true -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [stag1] s1=med_Led  &color1=1& hw_state=0 & (env_state1=2 |env_state1=1)  & (sound=1 | move=1)& Rec_motif1=false & staggered=true-> (s1'=low_Led);// Priority 3: environment sensing
  [stag1] s1=med_Led  &color1=1& hw_state=0 & (sound=0 | move=0)& Rec_motif1=false & staggered=true-> (q_r_high_c1*gamma)*beta1 : (s1'=finish) + 1-((q_r_high_c1*gamma)*beta1) : (s1'=low_Led);// default (empirical)
  [stag1] s1=med_Led  &color1=1& hw_state=0 & (move=1)& Rec_motif1=false & staggered=true-> 1-((q_r_high_c1*gamma)*beta1) : (s1'=task_failed) + (q_r_high_c1*gamma)*beta1 : (s1'=low_Led);// when movement un-intentionally

//low led white
  [stag1] s1=low_Led &color1=3& (hw_state=2 |env_state1=2) & Rec_motif1=false & staggered=true-> 1-((q_w_high_c1*gamma)*beta2) : (s1'=finish) + (q_w_high_c1*gamma)*beta2 : (s1'=task_failed); 
  [stag1] s1=low_Led &color1=3& (hw_state=0 |hw_state=1)& (env_state1=1 |env_state1=0)& Rec_motif1=false & staggered=true-> (q_w_high_c1*gamma)*beta2 : (s1'=finish) + 1-((q_w_high_c1*gamma)*beta2) : (s1'=task_failed);// default
//low led Blue
  [stag1] s1=low_Led &color1=2& (hw_state=2 |env_state1=2)& Rec_motif1=false & staggered=true -> 1-((q_b_high_c1*gamma)*beta2) : (s1'=finish) + (q_b_high_c1*gamma)*beta2 : (s1'=task_failed); 
  [stag1] s1=low_Led &color1=2& (hw_state=0 |hw_state=1)& (env_state1=1 |env_state1=0)& Rec_motif1=false & staggered=true-> (q_b_high_c1*gamma)*beta2 : (s1'=finish) + 1-((q_b_high_c1*gamma)*beta2) : (s1'=task_failed);// default
//low led Red
   [stag1] s1=low_Led &color1=1& (hw_state=2 |env_state1=2)& Rec_motif1=false & staggered=true -> 1-((q_r_high_c1*gamma)*beta2) : (s1'=finish) + (q_r_high_c1*gamma)*beta2 : (s1'=task_failed); 
  [stag1] s1=low_Led &color1=1& (hw_state=0 |hw_state=1)& (env_state1=1 |env_state1=0)& Rec_motif1=false & staggered=true-> (q_r_high_c1*gamma)*beta2 : (s1'=finish) + 1-((q_r_high_c1*gamma)*beta2) : (s1'=task_failed);// default
//////////////////////////////
  // Probabilistic transitions (BB1-specific)RECTANGLE pattern staggered with other clusters
///////////////////////////////
 //high pitch
  [stag1] s1=high_pitch & hw_state=2 & Rec_motif1=true & staggered=true -> (s1'=low_pitch); // Priority 1: hardware HIGH -> immediate downgrade
  [stag1] s1=high_pitch & hw_state=1 & Rec_motif1=true & staggered=true -> (s1'=med_pitch);// Priority 2: hardware MED -> reduce one level
  [stag1] s1=high_pitch & hw_state=0 & env_state1=2  & move=1& Rec_motif1=true & staggered=true ->(1-q_high_pitch_c1)/2 : (s1'=med_pitch) + 1-((1-q_high_pitch_c1)/2) : (s1'=low_pitch);// Priority 3: environment sensing
  [stag1] s1=high_pitch & hw_state=0 & env_state1= 0 & move=1& Rec_motif1=true & staggered=true->(1-q_high_pitch_c1) : (s1'=task_failed)+ q_high_pitch_c1/2 : (s1'=med_pitch) + q_high_pitch_c1/2 : (s1'=low_pitch);// when move (no direct intention of fallback), chance of failure occurs
  [stag1] s1=high_pitch & hw_state=0 & env_state1=1  & sound=1 & Rec_motif1=true & staggered=true->1-((1-q_high_pitch_c1)/2) : (s1'=med_pitch) + (1-q_high_pitch_c1)/2 : (s1'=low_pitch);
  [stag1] s1=high_pitch& hw_state=0   & (sound=0 & move=0) & Rec_motif1=true & staggered=true-> q_high_pitch_c1  : (s1'=finish) + (1-q_high_pitch_c1)/2 : (s1'=med_pitch) + (1-q_high_pitch_c1)/2 : (s1'=low_pitch); // default

//med pitch
  [stag1] s1=med_pitch & (hw_state=2 |hw_state=1) & Rec_motif1=true & staggered=true -> (s1'=low_pitch); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [stag1] s1=med_pitch & hw_state=0 & (env_state1=2 |env_state1=1)  &( sound=1 |move=1)& Rec_motif1=true & staggered=true -> (s1'=low_pitch);// Priority 3: environment sensing
  [stag1] s1=med_pitch & hw_state=0 & env_state1= 0 & move=1& Rec_motif1=true & staggered=true-> 1-(q_high_pitch_c1*beta1) : (s1'=task_failed)+ q_high_pitch_c1*beta1 : (s1'=low_pitch);// when move (no direct intention of fallback), chance of failure occurs
  [stag1] s1=med_pitch& hw_state=0  & (sound=0 & move=0)& Rec_motif1=true & staggered=true-> q_high_pitch_c1*beta1 : (s1'=finish) + (1-(q_high_pitch_c1*beta1)) : (s1'=low_pitch);// default

//low pitch
  [stag1] s1=low_pitch & (hw_state=2 |env_state1=2)& Rec_motif1=true & staggered=true -> (1-(q_high_pitch_c1*beta2)) : (s1'=finish) + q_high_pitch_c1*beta2 : (s1'=task_failed); 
  [stag1] s1=low_pitch & (hw_state=1 |env_state1=1)& Rec_motif1=true & staggered=true -> (1-((q_high_pitch_c1*beta2)/2)) : (s1'=finish) + (q_high_pitch_c1*beta2)/2 : (s1'=task_failed); 
  [stag1] s1=low_pitch & hw_state=0 & Rec_motif1=true & staggered=true-> q_high_pitch_c1*beta2 : (s1'=finish) + (1-(q_high_pitch_c1*beta2)) : (s1'=task_failed);//even if movement it is already very low consuming and it is the lowest state so failure exist but not as when a higher consuming state

//sensor-environment transition
  [stag1] s1=listen_on_moving_on & Rec_motif1=true & staggered=true-> 0.95 : (s1'=finish) + 0.05 : (s1'=task_failed);
  [stag1] s1=listen_on_moving_off & Rec_motif1=true & staggered=true-> 0.95 : (s1'=finish) + 0.05 : (s1'=task_failed);
  [stag1] s1=listen_off_moving_on& Rec_motif1=true & staggered=true -> 0.95 : (s1'=finish) + 0.05 : (s1'=task_failed);

//high-high simultaneous actuating white
  [stag1] s1=high_blink_high_buzz &color1=3 & hw_state=2 & Rec_motif1=true & staggered=true -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag1] s1=high_blink_high_buzz &color1=3 & hw_state=1 & Rec_motif1=true & staggered=true -> 0.5 : (s1'=high_blink_med_buzz) +0.5: (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level
  [stag1] s1=high_blink_high_buzz &color1=3 & hw_state=0 & env_state1=2  & move=1& Rec_motif1=true & staggered=true->0.5 : (s1'=med_blink_high_buzz) + 0.5 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag1] s1=high_blink_high_buzz &color1=3 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=true & staggered=true->1-(((q_w_high_c1*gamma)*theta)*q_high_pitch_c1) : (s1'=task_failed)+(((q_w_high_c1*gamma)*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + (((q_w_high_c1*gamma)*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [stag1] s1=high_blink_high_buzz &color1=3 & hw_state=0 & env_state1=1  & sound=1& Rec_motif1=true & staggered=true->((q_w_high_c1*gamma)*theta)*q_high_pitch_c1 : (s1'=high_blink_med_buzz) + 1-(((q_w_high_c1*gamma)*theta)*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [stag1] s1=high_blink_high_buzz &color1=3 & hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=true & staggered=true -> ((q_w_high_c1*gamma)*theta)*q_high_pitch_c1: (s1'=finish) + (1-(((q_w_high_c1*gamma)*theta)*q_high_pitch_c1))/3 : (s1'=high_blink_med_buzz) + (1-(((q_w_high_c1*gamma)*theta)*q_high_pitch_c1))/3 : (s1'=med_blink_high_buzz) + (1-(((q_w_high_c1*gamma)*theta)*q_high_pitch_c1))/3 : (s1'=med_blink_med_buzz); //default(Empirical)
//high-high simultaneous actuating blue
  [stag1] s1=high_blink_high_buzz &color1=2 & hw_state=2 & Rec_motif1=true & staggered=true -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag1] s1=high_blink_high_buzz &color1=2 & hw_state=1 & Rec_motif1=true & staggered=true -> 0.5 : (s1'=high_blink_med_buzz) +0.5: (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level
  [stag1] s1=high_blink_high_buzz &color1=2 & hw_state=0 & env_state1=2  & move=1& Rec_motif1=true & staggered=true->0.5 : (s1'=med_blink_high_buzz) + 0.5 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag1] s1=high_blink_high_buzz &color1=2 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=true & staggered=true->1-(((q_b_high_c1*gamma)*theta)*q_high_pitch_c1) : (s1'=task_failed)+(((q_b_high_c1*gamma)*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + (((q_b_high_c1*gamma)*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [stag1] s1=high_blink_high_buzz &color1=2 & hw_state=0 & env_state1=1  & sound=1& Rec_motif1=true & staggered=true->((q_b_high_c1*gamma)*theta)*q_high_pitch_c1 : (s1'=high_blink_med_buzz) + 1-(((q_b_high_c1*gamma)*theta)*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [stag1] s1=high_blink_high_buzz &color1=2 & hw_state=0 & env_state1=0  & (sound=0 & move=0) & Rec_motif1=true & staggered=true-> ((q_b_high_c1*gamma)*theta)*q_high_pitch_c1: (s1'=finish) + (1-(((q_b_high_c1*gamma)*theta)*q_high_pitch_c1))/3 : (s1'=high_blink_med_buzz) + (1-(((q_b_high_c1*gamma)*theta)*q_high_pitch_c1))/3 : (s1'=med_blink_high_buzz) + (1-(((q_b_high_c1*gamma)*theta)*q_high_pitch_c1))/3 : (s1'=med_blink_med_buzz); //default(Empirical)
//high-high simultaneous actuating RED
  [stag1] s1=high_blink_high_buzz &color1=1 & hw_state=2 & Rec_motif1=true & staggered=true -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag1] s1=high_blink_high_buzz &color1=1 & hw_state=1 & Rec_motif1=true & staggered=true -> 0.5 : (s1'=high_blink_med_buzz) +0.5: (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level
  [stag1] s1=high_blink_high_buzz &color1=1 & hw_state=0 & env_state1=2  & move=1& Rec_motif1=true & staggered=true->0.5 : (s1'=med_blink_high_buzz) + 0.5 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag1] s1=high_blink_high_buzz &color1=1 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=true & staggered=true->1-(((q_r_high_c1*gamma)*theta)*q_high_pitch_c1) : (s1'=task_failed)+(((q_r_high_c1*gamma)*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + (((q_r_high_c1*gamma)*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [stag1] s1=high_blink_high_buzz &color1=1 & hw_state=0 & env_state1=1  & sound=1& Rec_motif1=true & staggered=true->((q_r_high_c1*gamma)*theta)*q_high_pitch_c1 : (s1'=high_blink_med_buzz) + 1-(((q_r_high_c1*gamma)*theta)*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [stag1] s1=high_blink_high_buzz &color1=1 & hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=true & staggered=true -> ((q_r_high_c1*gamma)*theta)*q_high_pitch_c1: (s1'=finish) + (1-(((q_r_high_c1*gamma)*theta)*q_high_pitch_c1))/3 : (s1'=high_blink_med_buzz) + (1-(((q_r_high_c1*gamma)*theta)*q_high_pitch_c1))/3 : (s1'=med_blink_high_buzz) + (1-(((q_r_high_c1*gamma)*theta)*q_high_pitch_c1))/3 : (s1'=med_blink_med_buzz); //default(Empirical)

//high-med simultaneous actuating white
  [stag1] s1=high_blink_med_buzz &color1=3 & hw_state=2& Rec_motif1=true & staggered=true  -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag1] s1=high_blink_med_buzz &color1=3 & hw_state=1 & Rec_motif1=true & staggered=true -> 0.5 : (s1'=med_blink_med_buzz) + 0.5 : (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level /equal probabilities cause it deoends on user preferences
  [stag1] s1=high_blink_med_buzz &color1=3& hw_state=0 & env_state1=2  & move=1& Rec_motif1=true & staggered=true->1-(((q_w_high_c1*gamma)*theta)*q_high_pitch_c1) : (s1'=med_blink_high_buzz) + ((q_w_high_c1*gamma)*theta)*q_high_pitch_c1 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag1] s1=high_blink_high_buzz &color1=3 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=true & staggered=true->1-(((q_w_high_c1*gamma)*theta)*q_high_pitch_c1) : (s1'=task_failed)+(((q_w_high_c1*gamma)*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + (((q_w_high_c1*gamma)*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [stag1] s1=high_blink_med_buzz &color1=3& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=true & staggered=true->((q_w_high_c1*gamma)*theta)*q_high_pitch_c1  : (s1'=med_blink_med_buzz) + 1-(((q_w_high_c1*gamma)*theta)*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [stag1] s1=high_blink_med_buzz &color1=3& hw_state=0 & env_state1=0  & (sound=0 & move=0) & Rec_motif1=true & staggered=true-> ((q_w_high_c1*gamma)*theta)*q_high_pitch_c1 : (s1'=finish)  + (1-(((q_w_high_c1*gamma)*theta)*q_high_pitch_c1))/2 : (s1'=med_blink_high_buzz) + (1-(((q_w_high_c1*gamma)*theta)*q_high_pitch_c1))/2 : (s1'=med_blink_med_buzz); //default(empirical)
 //high-med simultaneous actuating blue
   [stag1] s1=high_blink_med_buzz &color1=2 & hw_state=2 & Rec_motif1=true & staggered=true -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag1] s1=high_blink_med_buzz &color1=2 & hw_state=1 & Rec_motif1=true & staggered=true -> 0.5 : (s1'=med_blink_med_buzz) + 0.5 : (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level /equal probabilities cause it deoends on user preferences
  [stag1] s1=high_blink_med_buzz &color1=2& hw_state=0 & env_state1=2  & move=1& Rec_motif1=true & staggered=true->1-(((q_b_high_c1*gamma)*theta)*q_high_pitch_c1) : (s1'=med_blink_high_buzz) + ((q_b_high_c1*gamma)*theta)*q_high_pitch_c1 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag1] s1=high_blink_high_buzz &color1=2 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=true & staggered=true->1-(((q_b_high_c1*gamma)*theta)*q_high_pitch_c1) : (s1'=task_failed)+(((q_b_high_c1*gamma)*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + (((q_b_high_c1*gamma)*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [stag1] s1=high_blink_med_buzz &color1=2& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=true & staggered=true->((q_b_high_c1*gamma)*theta)*q_high_pitch_c1  : (s1'=med_blink_med_buzz) + 1-(((q_b_high_c1*gamma)*theta)*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [stag1] s1=high_blink_med_buzz &color1=2& hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=true & staggered=true -> ((q_b_high_c1*gamma)*theta)*q_high_pitch_c1 : (s1'=finish)  + (1-(((q_b_high_c1*gamma)*theta)*q_high_pitch_c1))/2 : (s1'=med_blink_high_buzz) + (1-(((q_b_high_c1*gamma)*theta)*q_high_pitch_c1))/2 : (s1'=med_blink_med_buzz); //default(empirical)
 
//high-med simultaneous actuating Red
  [stag1] s1=high_blink_med_buzz &color1=1 & hw_state=2 & Rec_motif1=true & staggered=true -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag1] s1=high_blink_med_buzz &color1=1 & hw_state=1 & Rec_motif1=true & staggered=true -> 0.5 : (s1'=med_blink_med_buzz) + 0.5 : (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level /equal probabilities cause it deoends on user preferences
  [stag1] s1=high_blink_med_buzz &color1=1& hw_state=0 & env_state1=2  & move=1& Rec_motif1=true & staggered=true->1-(((q_r_high_c1*gamma)*theta)*q_high_pitch_c1) : (s1'=med_blink_high_buzz) + ((q_r_high_c1*gamma)*theta)*q_high_pitch_c1 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag1] s1=high_blink_high_buzz &color1=1 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=true & staggered=true->1-(((q_r_high_c1*gamma)*theta)*q_high_pitch_c1) : (s1'=task_failed)+(((q_r_high_c1*gamma)*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + (((q_r_high_c1*gamma)*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [stag1] s1=high_blink_med_buzz &color1=1& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=true & staggered=true->((q_r_high_c1*gamma)*theta)*q_high_pitch_c1  : (s1'=med_blink_med_buzz) + 1-(((q_r_high_c1*gamma)*theta)*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [stag1] s1=high_blink_med_buzz &color1=1& hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=true & staggered=true -> ((q_r_high_c1*gamma)*theta)*q_high_pitch_c1 : (s1'=finish)  + (1-(((q_r_high_c1*gamma)*theta)*q_high_pitch_c1))/2 : (s1'=med_blink_high_buzz) + (1-(((q_r_high_c1*gamma)*theta)*q_high_pitch_c1))/2 : (s1'=med_blink_med_buzz); //default(empirical)
 

 //med-high simultaneous actuating white
  [stag1] s1=med_blink_high_buzz &color1=3 & (hw_state=2 | hw_state=1) & Rec_motif1=true & staggered=true -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag1] s1=med_blink_high_buzz &color1=3 & hw_state=0 & (env_state1=2 | env_state1=1)   & (sound=1 | move=1)& Rec_motif1=true & staggered=true-> (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag1] s1=med_blink_high_buzz &color1=3 & hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=true & staggered=true-> ((q_w_high_c1*gamma)*theta)*q_high_pitch_c1*beta1 : (s1'=finish)  + 1-(((q_w_high_c1*gamma)*theta)*q_high_pitch_c1*beta1) : (s1'=med_blink_med_buzz); //default(empirical)
  [stag1] s1=med_blink_high_buzz &color1=3 & hw_state=0 & env_state1=0  & ( move=1)& Rec_motif1=true & staggered=true-> 1-(((q_w_high_c1*gamma)*theta)*q_high_pitch_c1*beta1) : (s1'=task_failed)  + ((q_w_high_c1*gamma)*theta)*q_high_pitch_c1*beta1 : (s1'=med_blink_med_buzz); //when movement un-intentionally
 //med-high simultaneous actuating blue
  [stag1] s1=med_blink_high_buzz &color1=2 & (hw_state=2 | hw_state=1)& Rec_motif1=true & staggered=true  -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag1] s1=med_blink_high_buzz &color1=2 & hw_state=0 & (env_state1=2 | env_state1=1)   & (sound=1 | move=1)& Rec_motif1=true & staggered=true-> (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag1] s1=med_blink_high_buzz &color1=2 & hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=true & staggered=true-> ((q_b_high_c1*gamma)*theta)*q_high_pitch_c1*beta1 : (s1'=finish)  + 1-(((q_b_high_c1*gamma)*theta)*q_high_pitch_c1*beta1) : (s1'=med_blink_med_buzz); //default(empirical)
  [stag1] s1=med_blink_high_buzz &color1=2 & hw_state=0 & env_state1=0  & ( move=1)& Rec_motif1=true & staggered=true-> 1-(((q_b_high_c1*gamma)*theta)*q_high_pitch_c1*beta1) : (s1'=task_failed)  + ((q_b_high_c1*gamma)*theta)*q_high_pitch_c1*beta1 : (s1'=med_blink_med_buzz); //when movement un-intentionally
 //med-high simultaneous actuating Red
  [stag1] s1=med_blink_high_buzz &color1=1 & (hw_state=2 | hw_state=1) & Rec_motif1=true & staggered=true -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag1] s1=med_blink_high_buzz &color1=1 & hw_state=0 & (env_state1=2 | env_state1=1)   & (sound=1 | move=1)& Rec_motif1=true & staggered=true-> (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag1] s1=med_blink_high_buzz &color1=1 & hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=true & staggered=true-> ((q_r_high_c1*gamma)*theta)*q_high_pitch_c1*beta1 : (s1'=finish)  + 1-(((q_r_high_c1*gamma)*theta)*q_high_pitch_c1*beta1) : (s1'=med_blink_med_buzz); //default(empirical)
  [stag1] s1=med_blink_high_buzz &color1=1 & hw_state=0 & env_state1=0  & ( move=1) & Rec_motif1=true & staggered=true-> 1-(((q_r_high_c1*gamma)*theta)*q_high_pitch_c1*beta1) : (s1'=task_failed)  + ((q_r_high_c1*gamma)*theta)*q_high_pitch_c1*beta1 : (s1'=med_blink_med_buzz); //when movement un-intentionally



//med-med simultaneous actuating white
  [stag1] s1=med_blink_med_buzz &color1=3& (hw_state=2 |env_state1=2)& Rec_motif1=true & staggered=true -> 1-(((q_w_high_c1*gamma)*theta)*q_high_pitch_c1*beta2) : (s1'=finish) + (((q_w_high_c1*gamma)*theta)*q_high_pitch_c1*beta2) : (s1'=task_failed); 
  [stag1] s1=med_blink_med_buzz &color1=3& (hw_state=1 |env_state1=1) & Rec_motif1=true & staggered=true-> 0.5 : (s1'=finish) + 0.5 : (s1'=task_failed); 
  [stag1] s1=med_blink_med_buzz &color1=3& hw_state=0& Rec_motif1=true & staggered=true-> (((q_w_high_c1*gamma)*theta)*q_high_pitch_c1*beta2) : (s1'=finish) + 1-(((q_w_high_c1*gamma)*theta)*q_high_pitch_c1*beta2) : (s1'=task_failed);//empirical data
//med-med simultaneous actuating Blue
[stag1] s1=med_blink_med_buzz &color1=2& (hw_state=2 |env_state1=2) & Rec_motif1=true & staggered=true-> 1-(((q_b_high_c1*gamma)*theta)*q_high_pitch_c1*beta2) : (s1'=finish) + (((q_b_high_c1*gamma)*theta)*q_high_pitch_c1*beta2) : (s1'=task_failed); 
  [stag1] s1=med_blink_med_buzz &color1=2& (hw_state=1 |env_state1=1)& Rec_motif1=true & staggered=true -> 0.5 : (s1'=finish) + 0.5 : (s1'=task_failed); 
  [stag1] s1=med_blink_med_buzz &color1=2& hw_state=0& Rec_motif1=true & staggered=true-> (((q_b_high_c1*gamma)*theta)*q_high_pitch_c1*beta2) : (s1'=finish) + 1-(((q_b_high_c1*gamma)*theta)*q_high_pitch_c1*beta2) : (s1'=task_failed);//empirical data
//med-med simultaneous actuating Red
  [stag1] s1=med_blink_med_buzz &color1=1& (hw_state=2 |env_state1=2)& Rec_motif1=true & staggered=true -> 1-(((q_r_high_c1*gamma)*theta)*q_high_pitch_c1*beta2) : (s1'=finish) + (((q_r_high_c1*gamma)*theta)*q_high_pitch_c1*beta2) : (s1'=task_failed); 
  [stag1] s1=med_blink_med_buzz &color1=1& (hw_state=1 |env_state1=1) & Rec_motif1=true & staggered=true-> 0.5 : (s1'=finish) + 0.5 : (s1'=task_failed); 
  [stag1] s1=med_blink_med_buzz &color1=1& hw_state=0 & Rec_motif1=true & staggered=true-> (((q_r_high_c1*gamma)*theta)*q_high_pitch_c1*beta2) : (s1'=finish) + 1-(((q_r_high_c1*gamma)*theta)*q_high_pitch_c1*beta2) : (s1'=task_failed);//empirical data

//High LED white
  [stag1] s1=high_Led  &color1=3& hw_state=2 & Rec_motif1=true & staggered=true -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade
  [stag1] s1=high_Led  &color1=3& hw_state=1 & Rec_motif1=true & staggered=true -> (s1'=med_Led);// Priority 2: hardware MED -> reduce one level
  [stag1] s1=high_Led  &color1=3& hw_state=0 & env_state1=2 & move=1& Rec_motif1=true & staggered=true->0.5  : (s1'=med_Led) + 0.5 : (s1'=low_Led);// Priority 3: environment sensing
  [stag1] s1=high_Led  &color1=3& hw_state=0 & env_state1=0 & move=1& Rec_motif1=true & staggered=true->(1-((q_w_high_c1*gamma)*theta)) : (s1'=task_failed)  +((q_w_high_c1*gamma)*theta)/2  : (s1'=med_Led) + ((q_w_high_c1*gamma)*theta)/2 : (s1'=low_Led);// when movement un-intentionally
  [stag1] s1=high_Led  &color1=3& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=true & staggered=true->0.5 : (s1'=med_Led) + 0.5 : (s1'=low_Led);
  [stag1] s1=high_Led  &color1=3& hw_state=0 & env_state1=0 & (sound=0 & move=0) & Rec_motif1=true & staggered=true-> ((q_w_high_c1*gamma)*theta) : (s1'=finish) + (1-((q_w_high_c1*gamma)*theta))/2 : (s1'=med_Led) + (1-((q_w_high_c1*gamma)*theta))/2 : (s1'=low_Led); // default (empirical)
//High LED Blue
  [stag1] s1=high_Led  &color1=2& hw_state=2 & Rec_motif1=true & staggered=true -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade
  [stag1] s1=high_Led  &color1=2& hw_state=1 & Rec_motif1=true & staggered=true -> (s1'=med_Led);// Priority 2: hardware MED -> reduce one level
  [stag1] s1=high_Led  &color1=2& hw_state=0 & env_state1=2 & move=1& Rec_motif1=true & staggered=true->0.5  : (s1'=med_Led) + 0.5 : (s1'=low_Led);// Priority 3: environment sensing
  [stag1] s1=high_Led  &color1=2& hw_state=0 & env_state1=0 & move=1& Rec_motif1=true & staggered=true->(1-((q_b_high_c1*gamma)*theta)) : (s1'=task_failed)  +((q_b_high_c1*gamma)*theta)/2  : (s1'=med_Led) + ((q_b_high_c1*gamma)*theta)/2 : (s1'=low_Led);// when movement un-intentionally
  [stag1] s1=high_Led  &color1=2& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=true & staggered=true->0.5 : (s1'=med_Led) + 0.5 : (s1'=low_Led);
  [stag1] s1=high_Led  &color1=2& hw_state=0 & env_state1=0 & (sound=0 & move=0)& Rec_motif1=true & staggered=true -> ((q_b_high_c1*gamma)*theta) : (s1'=finish) + (1-((q_b_high_c1*gamma)*theta))/2 : (s1'=med_Led) + (1-((q_b_high_c1*gamma)*theta))/2 : (s1'=low_Led); // default (empirical)
//High LED Red
   [stag1] s1=high_Led  &color1=1& hw_state=2 & Rec_motif1=true & staggered=true -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade
  [stag1] s1=high_Led  &color1=1& hw_state=1 & Rec_motif1=true & staggered=true -> (s1'=med_Led);// Priority 2: hardware MED -> reduce one level
  [stag1] s1=high_Led  &color1=1& hw_state=0 & env_state1=2 & move=1& Rec_motif1=true & staggered=true->0.5  : (s1'=med_Led) + 0.5 : (s1'=low_Led);// Priority 3: environment sensing
  [stag1] s1=high_Led  &color1=1& hw_state=0 & env_state1=0 & move=1& Rec_motif1=true & staggered=true->(1-((q_r_high_c1*gamma)*theta)) : (s1'=task_failed)  +((q_r_high_c1*gamma)*theta)/2  : (s1'=med_Led) + ((q_r_high_c1*gamma)*theta)/2 : (s1'=low_Led);// when movement un-intentionally
  [stag1] s1=high_Led  &color1=1& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=true & staggered=true->0.5 : (s1'=med_Led) + 0.5 : (s1'=low_Led);
  [stag1] s1=high_Led  &color1=1& hw_state=0 & env_state1=0 & (sound=0 & move=0)& Rec_motif1=true & staggered=true -> ((q_r_high_c1*gamma)*theta) : (s1'=finish) + (1-((q_r_high_c1*gamma)*theta))/2 : (s1'=med_Led) + (1-((q_r_high_c1*gamma)*theta))/2 : (s1'=low_Led); // default (empirical)

//Med LED white
  [stag1] s1=med_Led  &color1=3& (hw_state=2 |hw_state=1) & Rec_motif1=true & staggered=true -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [stag1] s1=med_Led  &color1=3& hw_state=0 & (env_state1=2 |env_state1=1)  & (sound=1 | move=1)& Rec_motif1=true & staggered=true-> (s1'=low_Led);// Priority 3: environment sensing
  [stag1] s1=med_Led  &color1=3& hw_state=0 & (sound=0 | move=0)& Rec_motif1=true & staggered=true-> ((q_w_high_c1*gamma)*theta)*beta1 : (s1'=finish) + 1-(((q_w_high_c1*gamma)*theta)*beta1) : (s1'=low_Led);// default (empirical)
  [stag1] s1=med_Led  &color1=3& hw_state=0 & (move=1)& Rec_motif1=true & staggered=true-> 1-(((q_w_high_c1*gamma)*theta)*beta1) : (s1'=task_failed) + ((q_w_high_c1*gamma)*theta)*beta1 : (s1'=low_Led);// when movement un-intentionally
//Med LED Blue
  [stag1] s1=med_Led   &color1=2& (hw_state=2 |hw_state=1) & Rec_motif1=true & staggered=true -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [stag1] s1=med_Led  &color1=2& hw_state=0 & (env_state1=2 |env_state1=1)  & (sound=1 | move=1)& Rec_motif1=true & staggered=true-> (s1'=low_Led);// Priority 3: environment sensing
  [stag1] s1=med_Led  &color1=2& hw_state=0 & (sound=0 | move=0)& Rec_motif1=true & staggered=true-> ((q_b_high_c1*gamma)*theta)*beta1 : (s1'=finish) + 1-(((q_b_high_c1*gamma)*theta)*beta1) : (s1'=low_Led);// default (empirical)
  [stag1] s1=med_Led  &color1=2& hw_state=0 & (move=1)& Rec_motif1=true & staggered=true-> 1-(((q_b_high_c1*gamma)*theta)*beta1) : (s1'=task_failed) + ((q_b_high_c1*gamma)*theta)*beta1 : (s1'=low_Led);// when movement un-intentionally
//Med LED Red
  [stag1] s1=med_Led   &color1=1& (hw_state=2 |hw_state=1) & Rec_motif1=true & staggered=true -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [stag1] s1=med_Led  &color1=1& hw_state=0 & (env_state1=2 |env_state1=1)  & (sound=1 | move=1)& Rec_motif1=true & staggered=true-> (s1'=low_Led);// Priority 3: environment sensing
  [stag1] s1=med_Led  &color1=1& hw_state=0 & (sound=0 | move=0)& Rec_motif1=true & staggered=true-> ((q_r_high_c1*gamma)*theta)*beta1 : (s1'=finish) + 1-(((q_r_high_c1*gamma)*theta)*beta1) : (s1'=low_Led);// default (empirical)
  [stag1] s1=med_Led  &color1=1& hw_state=0 & (move=1)& Rec_motif1=true & staggered=true-> 1-(((q_r_high_c1*gamma)*theta)*beta1) : (s1'=task_failed) + ((q_r_high_c1*gamma)*theta)*beta1 : (s1'=low_Led);// when movement un-intentionally

//low led white
  [stag1] s1=low_Led &color1=3& (hw_state=2 |env_state1=2) & Rec_motif1=true & staggered=true-> 1-(((q_w_high_c1*gamma)*theta)*beta2) : (s1'=finish) + ((q_w_high_c1*gamma)*theta)*beta2 : (s1'=task_failed); 
  [stag1] s1=low_Led &color1=3& (hw_state=0 |hw_state=1)& (env_state1=1 |env_state1=0)& Rec_motif1=true & staggered=true-> ((q_w_high_c1*gamma)*theta)*beta2 : (s1'=finish) + 1-(((q_w_high_c1*gamma)*theta)*beta2) : (s1'=task_failed);// default
//low led Blue
  [stag1] s1=low_Led &color1=2& (hw_state=2 |env_state1=2)& Rec_motif1=true & staggered=true -> 1-(((q_b_high_c1*gamma)*theta)*beta2) : (s1'=finish) + ((q_b_high_c1*gamma)*theta)*beta2 : (s1'=task_failed); 
  [stag1] s1=low_Led &color1=2& (hw_state=0 |hw_state=1)& (env_state1=1 |env_state1=0)& Rec_motif1=true & staggered=true-> ((q_b_high_c1*gamma)*theta)*beta2 : (s1'=finish) + 1-(((q_b_high_c1*gamma)*theta)*beta2) : (s1'=task_failed);// default
//low led Red
   [stag1] s1=low_Led &color1=1& (hw_state=2 |env_state1=2)& Rec_motif1=true & staggered=true -> 1-(((q_r_high_c1*gamma)*theta)*beta2) : (s1'=finish) + ((q_r_high_c1*gamma)*theta)*beta2 : (s1'=task_failed); 
  [stag1] s1=low_Led &color1=1& (hw_state=0 |hw_state=1)& (env_state1=1 |env_state1=0)& Rec_motif1=true & staggered=true-> ((q_r_high_c1*gamma)*theta)*beta2 : (s1'=finish) + 1-(((q_r_high_c1*gamma)*theta)*beta2) : (s1'=task_failed);// default
//////////////////////////////
//finish states 
  [turn1] s1=finish & turn=0 -> (s1'=start_task) ;
  [turn1] s1=task_failed & turn=0  -> (s1'=start_task) ;/// Not sure if i need to put the guard of turn=0 in all state transition when staggered=true?? Going to check this !!
endmodule
///////////////////////////////////////////////////////////////////////    CLUSTER2  ///////////////////////////////////////////////////////////////////
//: Cluster 2 of Blinky Blocks
module c2
  s2 : [0..19] init idle;
  color2 : [0..3] init 0;

//to reduce state explosion and to avoid non determinism from the beginning of the model
  [] (s2=idle) -> (s2'=start_task) ;
// dummy transition
  [step] (s2=start_task)& !active2 -> true ;

  // Start task (guarded by active1)
  [step] s2=start_task & active2 -> (s2'=choose_Red) & (color2'=1) ;
  [step] s2=start_task & active2 -> (s2'=choose_Blue) & (color2'=2) ;
  [step] s2=start_task & active2 -> (s2'=choose_white) & (color2'=3) ;
  [step] s2=start_task & active2 -> (s2'=high_pitch) ;
  [step] s2=start_task & active2 -> (s2'=med_pitch) ;
  [step] s2=start_task & active2 -> (s2'=low_pitch) ;
  [sense_both2] s2=start_task & active2 -> (s2'=listen_on_moving_on) ;
  [sense_mic2] s2=start_task & active2 -> (s2'=listen_on_moving_off) ;
  [sense_acc2] s2=start_task & active2 -> (s2'=listen_off_moving_on) ;

  // Color selection
  [step] s2=choose_Red -> (s2'=high_blink_high_buzz)  ;
  [step] s2=choose_Red -> (s2'=high_blink_med_buzz) ;
  [step] s2=choose_Red -> (s2'=med_blink_high_buzz) ;
  [step] s2=choose_Red -> (s2'=med_blink_med_buzz) ;
  [step] s2=choose_Red -> (s2'=high_Led) ;
  [step] s2=choose_Red -> (s2'=med_Led) ;
  [step] s2=choose_Red -> (s2'=low_Led) ;

  [step] s2=choose_Blue -> (s2'=high_blink_high_buzz) ;
  [step] s2=choose_Blue -> (s2'=high_blink_med_buzz) ;
  [step] s2=choose_Blue -> (s2'=med_blink_high_buzz) ;
  [step] s2=choose_Blue -> (s2'=med_blink_med_buzz) ;
  [step] s2=choose_Blue -> (s2'=high_Led) ;
  [step] s2=choose_Blue -> (s2'=med_Led) ;
  [step] s2=choose_Blue -> (s2'=low_Led) ;

  [step] s2=choose_white -> (s2'=high_blink_high_buzz) ;
  [step] s2=choose_white -> (s2'=high_blink_med_buzz) ;
  [step] s2=choose_white -> (s2'=med_blink_high_buzz) ;
  [step] s2=choose_white -> (s2'=med_blink_med_buzz) ;
  [step] s2=choose_white -> (s2'=high_Led) ;
  [step] s2=choose_white -> (s2'=med_Led) ;
  [step] s2=choose_white -> (s2'=low_Led) ;
////////////////////////////////////
  // Probabilistic transitions (BB1-specific) LINE pattern syncroninzed with other clusters
///////////////////////////////////
//high pitch
  [step] s2=high_pitch & hw_state=2 & Rec_motif2=false & staggered=false -> (s2'=low_pitch); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s2=high_pitch & hw_state=1 & Rec_motif2=false & staggered=false -> (s2'=med_pitch);// Priority 2: hardware MED -> reduce one level
  [step] s2=high_pitch & hw_state=0 & env_state2=2  & move=1& Rec_motif2=false & staggered=false ->(1-q_high_pitch_c1)/2 : (s2'=med_pitch) + 1-((1-q_high_pitch_c1)/2) : (s2'=low_pitch);// Priority 3: environment sensing
  [step] s2=high_pitch & hw_state=0 & env_state2= 0 & move=1& Rec_motif2=false & staggered=false->(1-q_high_pitch_c1) : (s2'=task_failed)+ q_high_pitch_c1/2 : (s2'=med_pitch) + q_high_pitch_c1/2 : (s2'=low_pitch);// when move (no direct intention of fallback), chance of failure occurs
  [step] s2=high_pitch & hw_state=0 & env_state2=1  & sound=1 & Rec_motif2=false & staggered=false->1-((1-q_high_pitch_c1)/2) : (s2'=med_pitch) + (1-q_high_pitch_c1)/2 : (s2'=low_pitch);
  [step] s2=high_pitch& hw_state=0   & (sound=0 & move=0) & Rec_motif2=false & staggered=false-> q_high_pitch_c1  : (s2'=finish) + (1-q_high_pitch_c1)/2 : (s2'=med_pitch) + (1-q_high_pitch_c1)/2 : (s2'=low_pitch); // default

//med pitch
  [step] s2=med_pitch & (hw_state=2 |hw_state=1) & Rec_motif2=false & staggered=false -> (s2'=low_pitch); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [step] s2=med_pitch & hw_state=0 & (env_state2=2 |env_state2=1)  &( sound=1 |move=1)& Rec_motif2=false & staggered=false -> (s2'=low_pitch);// Priority 3: environment sensing
  [step] s2=med_pitch & hw_state=0 & env_state2= 0 & move=1& Rec_motif2=false & staggered=false-> 1-(q_high_pitch_c1*beta1) : (s2'=task_failed)+ q_high_pitch_c1*beta1 : (s2'=low_pitch);// when move (no direct intention of fallback), chance of failure occurs
  [step] s2=med_pitch& hw_state=0  & (sound=0 & move=0)& Rec_motif2=false & staggered=false-> q_high_pitch_c1*beta1 : (s2'=finish) + (1-(q_high_pitch_c1*beta1)) : (s2'=low_pitch);// default

//low pitch
  [step] s2=low_pitch & (hw_state=2 |env_state2=2)& Rec_motif2=false & staggered=false -> (1-(q_high_pitch_c1*beta2)) : (s2'=finish) + q_high_pitch_c1*beta2 : (s2'=task_failed); 
  [step] s2=low_pitch & (hw_state=1 |env_state2=1)& Rec_motif2=false & staggered=false -> (1-((q_high_pitch_c1*beta2)/2)) : (s2'=finish) + (q_high_pitch_c1*beta2)/2 : (s2'=task_failed); 
  [step] s2=low_pitch & hw_state=0 & Rec_motif2=false & staggered=false-> q_high_pitch_c1*beta2 : (s2'=finish) + (1-(q_high_pitch_c1*beta2)) : (s2'=task_failed);//even if movement it is already very low consuming and it is the lowest state so failure exist but not as when a higher consuming state

//sensor-environment transition
  [step] s2=listen_on_moving_on & Rec_motif2=false & staggered=false-> 0.95 : (s2'=finish) + 0.05 : (s2'=task_failed);
  [step] s2=listen_on_moving_off & Rec_motif2=false & staggered=false-> 0.95 : (s2'=finish) + 0.05 : (s2'=task_failed);
  [step] s2=listen_off_moving_on& Rec_motif2=false & staggered=false -> 0.95 : (s2'=finish) + 0.05 : (s2'=task_failed);

//high-high simultaneous actuating white
  [step] s2=high_blink_high_buzz &color2=3 & hw_state=2 & Rec_motif2=false & staggered=false -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s2=high_blink_high_buzz &color2=3 & hw_state=1 & Rec_motif2=false & staggered=false -> 0.5 : (s2'=high_blink_med_buzz) +0.5: (s2'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level
  [step] s2=high_blink_high_buzz &color2=3 & hw_state=0 & env_state2=2  & move=1& Rec_motif2=false & staggered=false->0.5 : (s2'=med_blink_high_buzz) + 0.5 : (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s2=high_blink_high_buzz &color2=3 & hw_state=0 & env_state2=0  & move=1& Rec_motif2=false & staggered=false->1-((q_w_high_c1*alpha2)*q_high_pitch_c1) : (s2'=task_failed)+((q_w_high_c1*alpha2)*q_high_pitch_c1)/2 : (s2'=med_blink_high_buzz) + ((q_w_high_c1*alpha2)*q_high_pitch_c1)/2 : (s2'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s2=high_blink_high_buzz &color2=3 & hw_state=0 & env_state2=1  & sound=1& Rec_motif2=false & staggered=false->(q_w_high_c1*alpha2)*q_high_pitch_c1 : (s2'=high_blink_med_buzz) + 1-((q_w_high_c1*alpha2)*q_high_pitch_c1) : (s2'=med_blink_high_buzz);
  [step] s2=high_blink_high_buzz &color2=3 & hw_state=0 & env_state2=0  & (sound=0 & move=0)& Rec_motif2=false & staggered=false -> (q_w_high_c1*alpha2)*q_high_pitch_c1: (s2'=finish) + (1-((q_w_high_c1*alpha2)*q_high_pitch_c1))/3 : (s2'=high_blink_med_buzz) + (1-((q_w_high_c1*alpha2)*q_high_pitch_c1))/3 : (s2'=med_blink_high_buzz) + (1-((q_w_high_c1*alpha2)*q_high_pitch_c1))/3 : (s2'=med_blink_med_buzz); //default(Empirical)
//high-high simultaneous actuating blue
  [step] s2=high_blink_high_buzz &color2=2 & hw_state=2 & Rec_motif2=false & staggered=false -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s2=high_blink_high_buzz &color2=2 & hw_state=1 & Rec_motif2=false & staggered=false -> 0.5 : (s2'=high_blink_med_buzz) +0.5: (s2'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level
  [step] s2=high_blink_high_buzz &color2=2 & hw_state=0 & env_state2=2  & move=1& Rec_motif2=false & staggered=false->0.5 : (s2'=med_blink_high_buzz) + 0.5 : (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s2=high_blink_high_buzz &color2=2 & hw_state=0 & env_state2=0  & move=1& Rec_motif2=false & staggered=false->1-((q_b_high_c1*alpha2)*q_high_pitch_c1) : (s2'=task_failed)+((q_b_high_c1*alpha2)*q_high_pitch_c1)/2 : (s2'=med_blink_high_buzz) + ((q_b_high_c1*alpha2)*q_high_pitch_c1)/2 : (s2'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s2=high_blink_high_buzz &color2=2 & hw_state=0 & env_state2=1  & sound=1& Rec_motif2=false & staggered=false->(q_b_high_c1*alpha2)*q_high_pitch_c1 : (s2'=high_blink_med_buzz) + 1-((q_b_high_c1*alpha2)*q_high_pitch_c1) : (s2'=med_blink_high_buzz);
  [step] s2=high_blink_high_buzz &color2=2 & hw_state=0 & env_state2=0  & (sound=0 & move=0) & Rec_motif2=false & staggered=false-> (q_b_high_c1*alpha2)*q_high_pitch_c1: (s2'=finish) + (1-((q_b_high_c1*alpha2)*q_high_pitch_c1))/3 : (s2'=high_blink_med_buzz) + (1-((q_b_high_c1*alpha2)*q_high_pitch_c1))/3 : (s2'=med_blink_high_buzz) + (1-((q_b_high_c1*alpha2)*q_high_pitch_c1))/3 : (s2'=med_blink_med_buzz); //default(Empirical)
//high-high simultaneous actuating RED
  [step] s2=high_blink_high_buzz &color2=1 & hw_state=2 & Rec_motif2=false & staggered=false -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s2=high_blink_high_buzz &color2=1 & hw_state=1 & Rec_motif2=false & staggered=false -> 0.5 : (s2'=high_blink_med_buzz) +0.5: (s2'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level
  [step] s2=high_blink_high_buzz &color2=1 & hw_state=0 & env_state2=2  & move=1& Rec_motif2=false & staggered=false->0.5 : (s2'=med_blink_high_buzz) + 0.5 : (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s2=high_blink_high_buzz &color2=1 & hw_state=0 & env_state2=0  & move=1& Rec_motif2=false & staggered=false->1-((q_r_high_c1*alpha2)*q_high_pitch_c1) : (s2'=task_failed)+((q_r_high_c1*alpha2)*q_high_pitch_c1)/2 : (s2'=med_blink_high_buzz) + ((q_r_high_c1*alpha2)*q_high_pitch_c1)/2 : (s2'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s2=high_blink_high_buzz &color2=1 & hw_state=0 & env_state2=1  & sound=1& Rec_motif2=false & staggered=false->(q_r_high_c1*alpha2)*q_high_pitch_c1 : (s2'=high_blink_med_buzz) + 1-((q_r_high_c1*alpha2)*q_high_pitch_c1) : (s2'=med_blink_high_buzz);
  [step] s2=high_blink_high_buzz &color2=1 & hw_state=0 & env_state2=0  & (sound=0 & move=0)& Rec_motif2=false & staggered=false -> (q_r_high_c1*alpha2)*q_high_pitch_c1: (s2'=finish) + (1-((q_r_high_c1*alpha2)*q_high_pitch_c1))/3 : (s2'=high_blink_med_buzz) + (1-((q_r_high_c1*alpha2)*q_high_pitch_c1))/3 : (s2'=med_blink_high_buzz) + (1-((q_r_high_c1*alpha2)*q_high_pitch_c1))/3 : (s2'=med_blink_med_buzz); //default(Empirical)

//high-med simultaneous actuating white
  [step] s2=high_blink_med_buzz &color2=3 & hw_state=2& Rec_motif2=false & staggered=false  -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s2=high_blink_med_buzz &color2=3 & hw_state=1 & Rec_motif2=false & staggered=false -> 0.5 : (s2'=med_blink_med_buzz) + 0.5 : (s2'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level /equal probabilities cause it deoends on user preferences
  [step] s2=high_blink_med_buzz &color2=3& hw_state=0 & env_state2=2  & move=1& Rec_motif2=false & staggered=false->1-((q_w_high_c1*alpha2)*q_high_pitch_c1) : (s2'=med_blink_high_buzz) + (q_w_high_c1*alpha2)*q_high_pitch_c1 : (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s2=high_blink_high_buzz &color2=3 & hw_state=0 & env_state2=0  & move=1& Rec_motif2=false & staggered=false->1-((q_w_high_c1*alpha2)*q_high_pitch_c1) : (s2'=task_failed)+((q_w_high_c1*alpha2)*q_high_pitch_c1)/2 : (s2'=med_blink_high_buzz) + ((q_w_high_c1*alpha2)*q_high_pitch_c1)/2 : (s2'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s2=high_blink_med_buzz &color2=3& hw_state=0 & env_state2=1  & sound=1& Rec_motif2=false & staggered=false->(q_w_high_c1*alpha2)*q_high_pitch_c1  : (s2'=med_blink_med_buzz) + 1-((q_w_high_c1*alpha2)*q_high_pitch_c1) : (s2'=med_blink_high_buzz);
  [step] s2=high_blink_med_buzz &color2=3& hw_state=0 & env_state2=0  & (sound=0 & move=0) & Rec_motif2=false & staggered=false-> (q_w_high_c1*alpha2)*q_high_pitch_c1 : (s2'=finish)  + (1-((q_w_high_c1*alpha2)*q_high_pitch_c1))/2 : (s2'=med_blink_high_buzz) + (1-((q_w_high_c1*alpha2)*q_high_pitch_c1))/2 : (s2'=med_blink_med_buzz); //default(empirical)
 //high-med simultaneous actuating blue
   [step] s2=high_blink_med_buzz &color2=2 & hw_state=2 & Rec_motif2=false & staggered=false -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s2=high_blink_med_buzz &color2=2 & hw_state=1 & Rec_motif2=false & staggered=false -> 0.5 : (s2'=med_blink_med_buzz) + 0.5 : (s2'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level /equal probabilities cause it deoends on user preferences
  [step] s2=high_blink_med_buzz &color2=2& hw_state=0 & env_state2=2  & move=1& Rec_motif2=false & staggered=false->1-((q_b_high_c1*alpha2)*q_high_pitch_c1) : (s2'=med_blink_high_buzz) + (q_b_high_c1*alpha2)*q_high_pitch_c1 : (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s2=high_blink_high_buzz &color2=2 & hw_state=0 & env_state2=0  & move=1& Rec_motif2=false & staggered=false->1-((q_b_high_c1*alpha2)*q_high_pitch_c1) : (s2'=task_failed)+((q_b_high_c1*alpha2)*q_high_pitch_c1)/2 : (s2'=med_blink_high_buzz) + ((q_b_high_c1*alpha2)*q_high_pitch_c1)/2 : (s2'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s2=high_blink_med_buzz &color2=2& hw_state=0 & env_state2=1  & sound=1& Rec_motif2=false & staggered=false->(q_b_high_c1*alpha2)*q_high_pitch_c1  : (s2'=med_blink_med_buzz) + 1-((q_b_high_c1*alpha2)*q_high_pitch_c1) : (s2'=med_blink_high_buzz);
  [step] s2=high_blink_med_buzz &color2=2& hw_state=0 & env_state2=0  & (sound=0 & move=0)& Rec_motif2=false & staggered=false -> (q_b_high_c1*alpha2)*q_high_pitch_c1 : (s2'=finish)  + (1-((q_b_high_c1*alpha2)*q_high_pitch_c1))/2 : (s2'=med_blink_high_buzz) + (1-((q_b_high_c1*alpha2)*q_high_pitch_c1))/2 : (s2'=med_blink_med_buzz); //default(empirical)
 
//high-med simultaneous actuating Red
  [step] s2=high_blink_med_buzz &color2=1 & hw_state=2 & Rec_motif2=false & staggered=false -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s2=high_blink_med_buzz &color2=1 & hw_state=1 & Rec_motif2=false & staggered=false -> 0.5 : (s2'=med_blink_med_buzz) + 0.5 : (s2'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level /equal probabilities cause it deoends on user preferences
  [step] s2=high_blink_med_buzz &color2=1& hw_state=0 & env_state2=2  & move=1& Rec_motif2=false & staggered=false->1-((q_r_high_c1*alpha2)*q_high_pitch_c1) : (s2'=med_blink_high_buzz) + (q_r_high_c1*alpha2)*q_high_pitch_c1 : (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s2=high_blink_high_buzz &color2=1 & hw_state=0 & env_state2=0  & move=1& Rec_motif2=false & staggered=false->1-((q_r_high_c1*alpha2)*q_high_pitch_c1) : (s2'=task_failed)+((q_r_high_c1*alpha2)*q_high_pitch_c1)/2 : (s2'=med_blink_high_buzz) + ((q_r_high_c1*alpha2)*q_high_pitch_c1)/2 : (s2'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s2=high_blink_med_buzz &color2=1& hw_state=0 & env_state2=1  & sound=1& Rec_motif2=false & staggered=false->(q_r_high_c1*alpha2)*q_high_pitch_c1  : (s2'=med_blink_med_buzz) + 1-((q_r_high_c1*alpha2)*q_high_pitch_c1) : (s2'=med_blink_high_buzz);
  [step] s2=high_blink_med_buzz &color2=1& hw_state=0 & env_state2=0  & (sound=0 & move=0)& Rec_motif2=false & staggered=false -> (q_r_high_c1*alpha2)*q_high_pitch_c1 : (s2'=finish)  + (1-((q_r_high_c1*alpha2)*q_high_pitch_c1))/2 : (s2'=med_blink_high_buzz) + (1-((q_r_high_c1*alpha2)*q_high_pitch_c1))/2 : (s2'=med_blink_med_buzz); //default(empirical)
 

 //med-high simultaneous actuating white
  [step] s2=med_blink_high_buzz &color2=3 & (hw_state=2 | hw_state=1) & Rec_motif2=false & staggered=false -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s2=med_blink_high_buzz &color2=3 & hw_state=0 & (env_state2=2 | env_state2=1)   & (sound=1 | move=1)& Rec_motif2=false & staggered=false-> (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s2=med_blink_high_buzz &color2=3 & hw_state=0 & env_state2=0  & (sound=0 & move=0)& Rec_motif2=false & staggered=false-> (q_w_high_c1*alpha2)*q_high_pitch_c1*beta1 : (s2'=finish)  + 1-((q_w_high_c1*alpha2)*q_high_pitch_c1*beta1) : (s2'=med_blink_med_buzz); //default(empirical)
  [step] s2=med_blink_high_buzz &color2=3 & hw_state=0 & env_state2=0  & ( move=1)& Rec_motif2=false & staggered=false-> 1-((q_w_high_c1*alpha2)*q_high_pitch_c1*beta1) : (s2'=task_failed)  + (q_w_high_c1*alpha2)*q_high_pitch_c1*beta1 : (s2'=med_blink_med_buzz); //when movement un-intentionally
 //med-high simultaneous actuating blue
  [step] s2=med_blink_high_buzz &color2=2 & (hw_state=2 | hw_state=1)& Rec_motif2=false & staggered=false  -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s2=med_blink_high_buzz &color2=2 & hw_state=0 & (env_state2=2 | env_state2=1)   & (sound=1 | move=1)& Rec_motif2=false & staggered=false-> (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s2=med_blink_high_buzz &color2=2 & hw_state=0 & env_state2=0  & (sound=0 & move=0)& Rec_motif2=false & staggered=false-> (q_b_high_c1*alpha2)*q_high_pitch_c1*beta1 : (s2'=finish)  + 1-((q_b_high_c1*alpha2)*q_high_pitch_c1*beta1) : (s2'=med_blink_med_buzz); //default(empirical)
  [step] s2=med_blink_high_buzz &color2=2 & hw_state=0 & env_state2=0  & ( move=1)& Rec_motif2=false & staggered=false-> 1-((q_b_high_c1*alpha2)*q_high_pitch_c1*beta1) : (s2'=task_failed)  + (q_b_high_c1*alpha2)*q_high_pitch_c1*beta1 : (s2'=med_blink_med_buzz); //when movement un-intentionally
 //med-high simultaneous actuating Red
  [step] s2=med_blink_high_buzz &color2=1 & (hw_state=2 | hw_state=1) & Rec_motif2=false & staggered=false -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s2=med_blink_high_buzz &color2=1 & hw_state=0 & (env_state2=2 | env_state2=1)   & (sound=1 | move=1)& Rec_motif2=false & staggered=false-> (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s2=med_blink_high_buzz &color2=1 & hw_state=0 & env_state2=0  & (sound=0 & move=0)& Rec_motif2=false & staggered=false-> (q_r_high_c1*alpha2)*q_high_pitch_c1*beta1 : (s2'=finish)  + 1-((q_r_high_c1*alpha2)*q_high_pitch_c1*beta1) : (s2'=med_blink_med_buzz); //default(empirical)
  [step] s2=med_blink_high_buzz &color2=1 & hw_state=0 & env_state2=0  & ( move=1) & Rec_motif2=false & staggered=false-> 1-((q_r_high_c1*alpha2)*q_high_pitch_c1*beta1) : (s2'=task_failed)  + (q_r_high_c1*alpha2)*q_high_pitch_c1*beta1 : (s2'=med_blink_med_buzz); //when movement un-intentionally



//med-med simultaneous actuating white
  [step] s2=med_blink_med_buzz &color2=3& (hw_state=2 |env_state2=2)& Rec_motif2=false & staggered=false -> 1-((q_w_high_c1*alpha2)*q_high_pitch_c1*beta2) : (s2'=finish) + ((q_w_high_c1*alpha2)*q_high_pitch_c1*beta2) : (s2'=task_failed); 
  [step] s2=med_blink_med_buzz &color2=3& (hw_state=1 |env_state2=1) & Rec_motif2=false & staggered=false-> 0.5 : (s2'=finish) + 0.5 : (s2'=task_failed); 
  [step] s2=med_blink_med_buzz &color2=3& hw_state=0& Rec_motif2=false & staggered=false-> ((q_w_high_c1*alpha2)*q_high_pitch_c1*beta2) : (s2'=finish) + 1-((q_w_high_c1*alpha2)*q_high_pitch_c1*beta2) : (s2'=task_failed);//empirical data
//med-med simultaneous actuating Blue
[step] s2=med_blink_med_buzz &color2=2& (hw_state=2 |env_state2=2) & Rec_motif2=false & staggered=false-> 1-((q_b_high_c1*alpha2)*q_high_pitch_c1*beta2) : (s2'=finish) + ((q_b_high_c1*alpha2)*q_high_pitch_c1*beta2) : (s2'=task_failed); 
  [step] s2=med_blink_med_buzz &color2=2& (hw_state=1 |env_state2=1)& Rec_motif2=false & staggered=false -> 0.5 : (s2'=finish) + 0.5 : (s2'=task_failed); 
  [step] s2=med_blink_med_buzz &color2=2& hw_state=0& Rec_motif2=false & staggered=false-> ((q_b_high_c1*alpha2)*q_high_pitch_c1*beta2) : (s2'=finish) + 1-((q_b_high_c1*alpha2)*q_high_pitch_c1*beta2) : (s2'=task_failed);//empirical data
//med-med simultaneous actuating Red
  [step] s2=med_blink_med_buzz &color2=1& (hw_state=2 |env_state2=2)& Rec_motif2=false & staggered=false -> 1-((q_r_high_c1*alpha2)*q_high_pitch_c1*beta2) : (s2'=finish) + ((q_r_high_c1*alpha2)*q_high_pitch_c1*beta2) : (s2'=task_failed); 
  [step] s2=med_blink_med_buzz &color2=1& (hw_state=1 |env_state2=1) & Rec_motif2=false & staggered=false-> 0.5 : (s2'=finish) + 0.5 : (s2'=task_failed); 
  [step] s2=med_blink_med_buzz &color2=1& hw_state=0 & Rec_motif2=false & staggered=false-> ((q_r_high_c1*alpha2)*q_high_pitch_c1*beta2) : (s2'=finish) + 1-((q_r_high_c1*alpha2)*q_high_pitch_c1*beta2) : (s2'=task_failed);//empirical data

//High LED white
  [step] s2=high_Led  &color2=3& hw_state=2 & Rec_motif2=false & staggered=false -> (s2'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s2=high_Led  &color2=3& hw_state=1 & Rec_motif2=false & staggered=false -> (s2'=med_Led);// Priority 2: hardware MED -> reduce one level
  [step] s2=high_Led  &color2=3& hw_state=0 & env_state2=2 & move=1& Rec_motif2=false & staggered=false->0.5  : (s2'=med_Led) + 0.5 : (s2'=low_Led);// Priority 3: environment sensing
  [step] s2=high_Led  &color2=3& hw_state=0 & env_state2=0 & move=1& Rec_motif2=false & staggered=false->(1-(q_w_high_c1*alpha2)) : (s2'=task_failed)  +(q_w_high_c1*alpha2)/2  : (s2'=med_Led) + (q_w_high_c1*alpha2)/2 : (s2'=low_Led);// when movement un-intentionally
  [step] s2=high_Led  &color2=3& hw_state=0 & env_state2=1  & sound=1& Rec_motif2=false & staggered=false->0.5 : (s2'=med_Led) + 0.5 : (s2'=low_Led);
  [step] s2=high_Led  &color2=3& hw_state=0 & env_state2=0 & (sound=0 & move=0) & Rec_motif2=false & staggered=false-> (q_w_high_c1*alpha2) : (s2'=finish) + (1-(q_w_high_c1*alpha2))/2 : (s2'=med_Led) + (1-(q_w_high_c1*alpha2))/2 : (s2'=low_Led); // default (empirical)
//High LED Blue
  [step] s2=high_Led  &color2=2& hw_state=2 & Rec_motif2=false & staggered=false -> (s2'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s2=high_Led  &color2=2& hw_state=1 & Rec_motif2=false & staggered=false -> (s2'=med_Led);// Priority 2: hardware MED -> reduce one level
  [step] s2=high_Led  &color2=2& hw_state=0 & env_state2=2 & move=1& Rec_motif2=false & staggered=false->0.5  : (s2'=med_Led) + 0.5 : (s2'=low_Led);// Priority 3: environment sensing
  [step] s2=high_Led  &color2=2& hw_state=0 & env_state2=0 & move=1& Rec_motif2=false & staggered=false->(1-(q_b_high_c1*alpha2)) : (s2'=task_failed)  +(q_b_high_c1*alpha2)/2  : (s2'=med_Led) + (q_b_high_c1*alpha2)/2 : (s2'=low_Led);// when movement un-intentionally
  [step] s2=high_Led  &color2=2& hw_state=0 & env_state2=1  & sound=1& Rec_motif2=false & staggered=false->0.5 : (s2'=med_Led) + 0.5 : (s2'=low_Led);
  [step] s2=high_Led  &color2=2& hw_state=0 & env_state2=0 & (sound=0 & move=0)& Rec_motif2=false & staggered=false -> (q_b_high_c1*alpha2) : (s2'=finish) + (1-(q_b_high_c1*alpha2))/2 : (s2'=med_Led) + (1-(q_b_high_c1*alpha2))/2 : (s2'=low_Led); // default (empirical)
//High LED Red
   [step] s2=high_Led  &color2=1& hw_state=2 & Rec_motif2=false & staggered=false -> (s2'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s2=high_Led  &color2=1& hw_state=1 & Rec_motif2=false & staggered=false -> (s2'=med_Led);// Priority 2: hardware MED -> reduce one level
  [step] s2=high_Led  &color2=1& hw_state=0 & env_state2=2 & move=1& Rec_motif2=false & staggered=false->0.5  : (s2'=med_Led) + 0.5 : (s2'=low_Led);// Priority 3: environment sensing
  [step] s2=high_Led  &color2=1& hw_state=0 & env_state2=0 & move=1& Rec_motif2=false & staggered=false->(1-(q_r_high_c1*alpha2)) : (s2'=task_failed)  +(q_r_high_c1*alpha2)/2  : (s2'=med_Led) + (q_r_high_c1*alpha2)/2 : (s2'=low_Led);// when movement un-intentionally
  [step] s2=high_Led  &color2=1& hw_state=0 & env_state2=1  & sound=1& Rec_motif2=false & staggered=false->0.5 : (s2'=med_Led) + 0.5 : (s2'=low_Led);
  [step] s2=high_Led  &color2=1& hw_state=0 & env_state2=0 & (sound=0 & move=0)& Rec_motif2=false & staggered=false -> (q_r_high_c1*alpha2) : (s2'=finish) + (1-(q_r_high_c1*alpha2))/2 : (s2'=med_Led) + (1-(q_r_high_c1*alpha2))/2 : (s2'=low_Led); // default (empirical)

//Med LED white
  [step] s2=med_Led  &color2=3& (hw_state=2 |hw_state=1) & Rec_motif2=false & staggered=false -> (s2'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [step] s2=med_Led  &color2=3& hw_state=0 & (env_state2=2 |env_state2=1)  & (sound=1 | move=1)& Rec_motif2=false & staggered=false-> (s2'=low_Led);// Priority 3: environment sensing
  [step] s2=med_Led  &color2=3& hw_state=0 & (sound=0 | move=0)& Rec_motif2=false & staggered=false-> (q_w_high_c1*alpha2)*beta1 : (s2'=finish) + 1-((q_w_high_c1*alpha2)*beta1) : (s2'=low_Led);// default (empirical)
  [step] s2=med_Led  &color2=3& hw_state=0 & (move=1)& Rec_motif2=false & staggered=false-> 1-((q_w_high_c1*alpha2)*beta1) : (s2'=task_failed) + (q_w_high_c1*alpha2)*beta1 : (s2'=low_Led);// when movement un-intentionally
//Med LED Blue
  [step] s2=med_Led   &color2=2& (hw_state=2 |hw_state=1) & Rec_motif2=false & staggered=false -> (s2'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [step] s2=med_Led  &color2=2& hw_state=0 & (env_state2=2 |env_state2=1)  & (sound=1 | move=1)& Rec_motif2=false & staggered=false-> (s2'=low_Led);// Priority 3: environment sensing
  [step] s2=med_Led  &color2=2& hw_state=0 & (sound=0 | move=0)& Rec_motif2=false & staggered=false-> (q_b_high_c1*alpha2)*beta1 : (s2'=finish) + 1-((q_b_high_c1*alpha2)*beta1) : (s2'=low_Led);// default (empirical)
  [step] s2=med_Led  &color2=2& hw_state=0 & (move=1)& Rec_motif2=false & staggered=false-> 1-((q_b_high_c1*alpha2)*beta1) : (s2'=task_failed) + (q_b_high_c1*alpha2)*beta1 : (s2'=low_Led);// when movement un-intentionally
//Med LED Red
  [step] s2=med_Led   &color2=1& (hw_state=2 |hw_state=1) & Rec_motif2=false & staggered=false -> (s2'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [step] s2=med_Led  &color2=1& hw_state=0 & (env_state2=2 |env_state2=1)  & (sound=1 | move=1)& Rec_motif2=false & staggered=false-> (s2'=low_Led);// Priority 3: environment sensing
  [step] s2=med_Led  &color2=1& hw_state=0 & (sound=0 | move=0)& Rec_motif2=false & staggered=false-> (q_r_high_c1*alpha2)*beta1 : (s2'=finish) + 1-((q_r_high_c1*alpha2)*beta1) : (s2'=low_Led);// default (empirical)
  [step] s2=med_Led  &color2=1& hw_state=0 & (move=1)& Rec_motif2=false & staggered=false-> 1-((q_r_high_c1*alpha2)*beta1) : (s2'=task_failed) + (q_r_high_c1*alpha2)*beta1 : (s2'=low_Led);// when movement un-intentionally

//low led white
  [step] s2=low_Led &color2=3& (hw_state=2 |env_state2=2) & Rec_motif2=false & staggered=false-> 1-((q_w_high_c1*alpha2)*beta2) : (s2'=finish) + (q_w_high_c1*alpha2)*beta2 : (s2'=task_failed); 
  [step] s2=low_Led &color2=3& (hw_state=0 |hw_state=1)& (env_state2=1 |env_state2=0)& Rec_motif2=false & staggered=false-> (q_w_high_c1*alpha2)*beta2 : (s2'=finish) + 1-((q_w_high_c1*alpha2)*beta2) : (s2'=task_failed);// default
//low led Blue
  [step] s2=low_Led &color2=2& (hw_state=2 |env_state2=2)& Rec_motif2=false & staggered=false -> 1-((q_b_high_c1*alpha2)*beta2) : (s2'=finish) + (q_b_high_c1*alpha2)*beta2 : (s2'=task_failed); 
  [step] s2=low_Led &color2=2& (hw_state=0 |hw_state=1)& (env_state2=1 |env_state2=0)& Rec_motif2=false & staggered=false-> (q_b_high_c1*alpha2)*beta2 : (s2'=finish) + 1-((q_b_high_c1*alpha2)*beta2) : (s2'=task_failed);// default
//low led Red
   [step] s2=low_Led &color2=1& (hw_state=2 |env_state2=2)& Rec_motif2=false & staggered=false -> 1-((q_r_high_c1*alpha2)*beta2) : (s2'=finish) + (q_r_high_c1*alpha2)*beta2 : (s2'=task_failed); 
  [step] s2=low_Led &color2=1& (hw_state=0 |hw_state=1)& (env_state2=1 |env_state2=0)& Rec_motif2=false & staggered=false-> (q_r_high_c1*alpha2)*beta2 : (s2'=finish) + 1-((q_r_high_c1*alpha2)*beta2) : (s2'=task_failed);// default
//////////////////////////////
  // Probabilistic transitions (BB1-specific)RECTANGLE pattern syncroninzed with other clusters
///////////////////////////////
 //high pitch
  [step] s2=high_pitch & hw_state=2 & Rec_motif2=true & staggered=false -> (s2'=low_pitch); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s2=high_pitch & hw_state=1 & Rec_motif2=true & staggered=false -> (s2'=med_pitch);// Priority 2: hardware MED -> reduce one level
  [step] s2=high_pitch & hw_state=0 & env_state2=2  & move=1& Rec_motif2=true & staggered=false ->(1-q_high_pitch_c1)/2 : (s2'=med_pitch) + 1-((1-q_high_pitch_c1)/2) : (s2'=low_pitch);// Priority 3: environment sensing
  [step] s2=high_pitch & hw_state=0 & env_state2= 0 & move=1& Rec_motif2=true & staggered=false->(1-q_high_pitch_c1) : (s2'=task_failed)+ q_high_pitch_c1/2 : (s2'=med_pitch) + q_high_pitch_c1/2 : (s2'=low_pitch);// when move (no direct intention of fallback), chance of failure occurs
  [step] s2=high_pitch & hw_state=0 & env_state2=1  & sound=1 & Rec_motif2=true & staggered=false->1-((1-q_high_pitch_c1)/2) : (s2'=med_pitch) + (1-q_high_pitch_c1)/2 : (s2'=low_pitch);
  [step] s2=high_pitch& hw_state=0   & (sound=0 & move=0) & Rec_motif2=true & staggered=false-> q_high_pitch_c1  : (s2'=finish) + (1-q_high_pitch_c1)/2 : (s2'=med_pitch) + (1-q_high_pitch_c1)/2 : (s2'=low_pitch); // default

//med pitch
  [step] s2=med_pitch & (hw_state=2 |hw_state=1) & Rec_motif2=true & staggered=false -> (s2'=low_pitch); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [step] s2=med_pitch & hw_state=0 & (env_state2=2 |env_state2=1)  &( sound=1 |move=1)& Rec_motif2=true & staggered=false -> (s2'=low_pitch);// Priority 3: environment sensing
  [step] s2=med_pitch & hw_state=0 & env_state2= 0 & move=1& Rec_motif2=true & staggered=false-> 1-(q_high_pitch_c1*beta1) : (s2'=task_failed)+ q_high_pitch_c1*beta1 : (s2'=low_pitch);// when move (no direct intention of fallback), chance of failure occurs
  [step] s2=med_pitch& hw_state=0  & (sound=0 & move=0)& Rec_motif2=true & staggered=false-> q_high_pitch_c1*beta1 : (s2'=finish) + (1-(q_high_pitch_c1*beta1)) : (s2'=low_pitch);// default

//low pitch
  [step] s2=low_pitch & (hw_state=2 |env_state2=2)& Rec_motif2=true & staggered=false -> (1-(q_high_pitch_c1*beta2)) : (s2'=finish) + q_high_pitch_c1*beta2 : (s2'=task_failed); 
  [step] s2=low_pitch & (hw_state=1 |env_state2=1)& Rec_motif2=true & staggered=false -> (1-((q_high_pitch_c1*beta2)/2)) : (s2'=finish) + (q_high_pitch_c1*beta2)/2 : (s2'=task_failed); 
  [step] s2=low_pitch & hw_state=0 & Rec_motif2=true & staggered=false-> q_high_pitch_c1*beta2 : (s2'=finish) + (1-(q_high_pitch_c1*beta2)) : (s2'=task_failed);//even if movement it is already very low consuming and it is the lowest state so failure exist but not as when a higher consuming state

//sensor-environment transition
  [step] s2=listen_on_moving_on & Rec_motif2=true & staggered=false-> 0.95 : (s2'=finish) + 0.05 : (s2'=task_failed);
  [step] s2=listen_on_moving_off & Rec_motif2=true & staggered=false-> 0.95 : (s2'=finish) + 0.05 : (s2'=task_failed);
  [step] s2=listen_off_moving_on& Rec_motif2=true & staggered=false -> 0.95 : (s2'=finish) + 0.05 : (s2'=task_failed);

//high-high simultaneous actuating white
  [step] s2=high_blink_high_buzz &color2=3 & hw_state=2 & Rec_motif2=true & staggered=false -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s2=high_blink_high_buzz &color2=3 & hw_state=1 & Rec_motif2=true & staggered=false -> 0.5 : (s2'=high_blink_med_buzz) +0.5: (s2'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level
  [step] s2=high_blink_high_buzz &color2=3 & hw_state=0 & env_state2=2  & move=1& Rec_motif2=true & staggered=false->0.5 : (s2'=med_blink_high_buzz) + 0.5 : (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s2=high_blink_high_buzz &color2=3 & hw_state=0 & env_state2=0  & move=1& Rec_motif2=true & staggered=false->1-(((q_w_high_c1*alpha2)*theta)*q_high_pitch_c1) : (s2'=task_failed)+(((q_w_high_c1*alpha2)*theta)*q_high_pitch_c1)/2 : (s2'=med_blink_high_buzz) + (((q_w_high_c1*alpha2)*theta)*q_high_pitch_c1)/2 : (s2'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s2=high_blink_high_buzz &color2=3 & hw_state=0 & env_state2=1  & sound=1& Rec_motif2=true & staggered=false->((q_w_high_c1*alpha2)*theta)*q_high_pitch_c1 : (s2'=high_blink_med_buzz) + 1-(((q_w_high_c1*alpha2)*theta)*q_high_pitch_c1) : (s2'=med_blink_high_buzz);
  [step] s2=high_blink_high_buzz &color2=3 & hw_state=0 & env_state2=0  & (sound=0 & move=0)& Rec_motif2=true & staggered=false -> ((q_w_high_c1*alpha2)*theta)*q_high_pitch_c1: (s2'=finish) + (1-(((q_w_high_c1*alpha2)*theta)*q_high_pitch_c1))/3 : (s2'=high_blink_med_buzz) + (1-(((q_w_high_c1*alpha2)*theta)*q_high_pitch_c1))/3 : (s2'=med_blink_high_buzz) + (1-(((q_w_high_c1*alpha2)*theta)*q_high_pitch_c1))/3 : (s2'=med_blink_med_buzz); //default(Empirical)
//high-high simultaneous actuating blue
  [step] s2=high_blink_high_buzz &color2=2 & hw_state=2 & Rec_motif2=true & staggered=false -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s2=high_blink_high_buzz &color2=2 & hw_state=1 & Rec_motif2=true & staggered=false -> 0.5 : (s2'=high_blink_med_buzz) +0.5: (s2'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level
  [step] s2=high_blink_high_buzz &color2=2 & hw_state=0 & env_state2=2  & move=1& Rec_motif2=true & staggered=false->0.5 : (s2'=med_blink_high_buzz) + 0.5 : (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s2=high_blink_high_buzz &color2=2 & hw_state=0 & env_state2=0  & move=1& Rec_motif2=true & staggered=false->1-(((q_b_high_c1*alpha2)*theta)*q_high_pitch_c1) : (s2'=task_failed)+(((q_b_high_c1*alpha2)*theta)*q_high_pitch_c1)/2 : (s2'=med_blink_high_buzz) + (((q_b_high_c1*alpha2)*theta)*q_high_pitch_c1)/2 : (s2'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s2=high_blink_high_buzz &color2=2 & hw_state=0 & env_state2=1  & sound=1& Rec_motif2=true & staggered=false->((q_b_high_c1*alpha2)*theta)*q_high_pitch_c1 : (s2'=high_blink_med_buzz) + 1-(((q_b_high_c1*alpha2)*theta)*q_high_pitch_c1) : (s2'=med_blink_high_buzz);
  [step] s2=high_blink_high_buzz &color2=2 & hw_state=0 & env_state2=0  & (sound=0 & move=0) & Rec_motif2=true & staggered=false-> ((q_b_high_c1*alpha2)*theta)*q_high_pitch_c1: (s2'=finish) + (1-(((q_b_high_c1*alpha2)*theta)*q_high_pitch_c1))/3 : (s2'=high_blink_med_buzz) + (1-(((q_b_high_c1*alpha2)*theta)*q_high_pitch_c1))/3 : (s2'=med_blink_high_buzz) + (1-(((q_b_high_c1*alpha2)*theta)*q_high_pitch_c1))/3 : (s2'=med_blink_med_buzz); //default(Empirical)
//high-high simultaneous actuating RED
  [step] s2=high_blink_high_buzz &color2=1 & hw_state=2 & Rec_motif2=true & staggered=false -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s2=high_blink_high_buzz &color2=1 & hw_state=1 & Rec_motif2=true & staggered=false -> 0.5 : (s2'=high_blink_med_buzz) +0.5: (s2'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level
  [step] s2=high_blink_high_buzz &color2=1 & hw_state=0 & env_state2=2  & move=1& Rec_motif2=true & staggered=false->0.5 : (s2'=med_blink_high_buzz) + 0.5 : (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s2=high_blink_high_buzz &color2=1 & hw_state=0 & env_state2=0  & move=1& Rec_motif2=true & staggered=false->1-(((q_r_high_c1*alpha2)*theta)*q_high_pitch_c1) : (s2'=task_failed)+(((q_r_high_c1*alpha2)*theta)*q_high_pitch_c1)/2 : (s2'=med_blink_high_buzz) + (((q_r_high_c1*alpha2)*theta)*q_high_pitch_c1)/2 : (s2'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s2=high_blink_high_buzz &color2=1 & hw_state=0 & env_state2=1  & sound=1& Rec_motif2=true & staggered=false->((q_r_high_c1*alpha2)*theta)*q_high_pitch_c1 : (s2'=high_blink_med_buzz) + 1-(((q_r_high_c1*alpha2)*theta)*q_high_pitch_c1) : (s2'=med_blink_high_buzz);
  [step] s2=high_blink_high_buzz &color2=1 & hw_state=0 & env_state2=0  & (sound=0 & move=0)& Rec_motif2=true & staggered=false -> ((q_r_high_c1*alpha2)*theta)*q_high_pitch_c1: (s2'=finish) + (1-(((q_r_high_c1*alpha2)*theta)*q_high_pitch_c1))/3 : (s2'=high_blink_med_buzz) + (1-(((q_r_high_c1*alpha2)*theta)*q_high_pitch_c1))/3 : (s2'=med_blink_high_buzz) + (1-(((q_r_high_c1*alpha2)*theta)*q_high_pitch_c1))/3 : (s2'=med_blink_med_buzz); //default(Empirical)

//high-med simultaneous actuating white
  [step] s2=high_blink_med_buzz &color2=3 & hw_state=2& Rec_motif2=true & staggered=false  -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s2=high_blink_med_buzz &color2=3 & hw_state=1 & Rec_motif2=true & staggered=false -> 0.5 : (s2'=med_blink_med_buzz) + 0.5 : (s2'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level /equal probabilities cause it deoends on user preferences
  [step] s2=high_blink_med_buzz &color2=3& hw_state=0 & env_state2=2  & move=1& Rec_motif2=true & staggered=false->1-(((q_w_high_c1*alpha2)*theta)*q_high_pitch_c1) : (s2'=med_blink_high_buzz) + ((q_w_high_c1*alpha2)*theta)*q_high_pitch_c1 : (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s2=high_blink_high_buzz &color2=3 & hw_state=0 & env_state2=0  & move=1& Rec_motif2=true & staggered=false->1-(((q_w_high_c1*alpha2)*theta)*q_high_pitch_c1) : (s2'=task_failed)+(((q_w_high_c1*alpha2)*theta)*q_high_pitch_c1)/2 : (s2'=med_blink_high_buzz) + (((q_w_high_c1*alpha2)*theta)*q_high_pitch_c1)/2 : (s2'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s2=high_blink_med_buzz &color2=3& hw_state=0 & env_state2=1  & sound=1& Rec_motif2=true & staggered=false->((q_w_high_c1*alpha2)*theta)*q_high_pitch_c1  : (s2'=med_blink_med_buzz) + 1-(((q_w_high_c1*alpha2)*theta)*q_high_pitch_c1) : (s2'=med_blink_high_buzz);
  [step] s2=high_blink_med_buzz &color2=3& hw_state=0 & env_state2=0  & (sound=0 & move=0) & Rec_motif2=true & staggered=false-> ((q_w_high_c1*alpha2)*theta)*q_high_pitch_c1 : (s2'=finish)  + (1-(((q_w_high_c1*alpha2)*theta)*q_high_pitch_c1))/2 : (s2'=med_blink_high_buzz) + (1-(((q_w_high_c1*alpha2)*theta)*q_high_pitch_c1))/2 : (s2'=med_blink_med_buzz); //default(empirical)
 //high-med simultaneous actuating blue
   [step] s2=high_blink_med_buzz &color2=2 & hw_state=2 & Rec_motif2=true & staggered=false -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s2=high_blink_med_buzz &color2=2 & hw_state=1 & Rec_motif2=true & staggered=false -> 0.5 : (s2'=med_blink_med_buzz) + 0.5 : (s2'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level /equal probabilities cause it deoends on user preferences
  [step] s2=high_blink_med_buzz &color2=2& hw_state=0 & env_state2=2  & move=1& Rec_motif2=true & staggered=false->1-(((q_b_high_c1*alpha2)*theta)*q_high_pitch_c1) : (s2'=med_blink_high_buzz) + ((q_b_high_c1*alpha2)*theta)*q_high_pitch_c1 : (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s2=high_blink_high_buzz &color2=2 & hw_state=0 & env_state2=0  & move=1& Rec_motif2=true & staggered=false->1-(((q_b_high_c1*alpha2)*theta)*q_high_pitch_c1) : (s2'=task_failed)+(((q_b_high_c1*alpha2)*theta)*q_high_pitch_c1)/2 : (s2'=med_blink_high_buzz) + (((q_b_high_c1*alpha2)*theta)*q_high_pitch_c1)/2 : (s2'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s2=high_blink_med_buzz &color2=2& hw_state=0 & env_state2=1  & sound=1& Rec_motif2=true & staggered=false->((q_b_high_c1*alpha2)*theta)*q_high_pitch_c1  : (s2'=med_blink_med_buzz) + 1-(((q_b_high_c1*alpha2)*theta)*q_high_pitch_c1) : (s2'=med_blink_high_buzz);
  [step] s2=high_blink_med_buzz &color2=2& hw_state=0 & env_state2=0  & (sound=0 & move=0)& Rec_motif2=true & staggered=false -> ((q_b_high_c1*alpha2)*theta)*q_high_pitch_c1 : (s2'=finish)  + (1-(((q_b_high_c1*alpha2)*theta)*q_high_pitch_c1))/2 : (s2'=med_blink_high_buzz) + (1-(((q_b_high_c1*alpha2)*theta)*q_high_pitch_c1))/2 : (s2'=med_blink_med_buzz); //default(empirical)
 
//high-med simultaneous actuating Red
  [step] s2=high_blink_med_buzz &color2=1 & hw_state=2 & Rec_motif2=true & staggered=false -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s2=high_blink_med_buzz &color2=1 & hw_state=1 & Rec_motif2=true & staggered=false -> 0.5 : (s2'=med_blink_med_buzz) + 0.5 : (s2'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level /equal probabilities cause it deoends on user preferences
  [step] s2=high_blink_med_buzz &color2=1& hw_state=0 & env_state2=2  & move=1& Rec_motif2=true & staggered=false->1-(((q_r_high_c1*alpha2)*theta)*q_high_pitch_c1) : (s2'=med_blink_high_buzz) + ((q_r_high_c1*alpha2)*theta)*q_high_pitch_c1 : (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s2=high_blink_high_buzz &color2=1 & hw_state=0 & env_state2=0  & move=1& Rec_motif2=true & staggered=false->1-(((q_r_high_c1*alpha2)*theta)*q_high_pitch_c1) : (s2'=task_failed)+(((q_r_high_c1*alpha2)*theta)*q_high_pitch_c1)/2 : (s2'=med_blink_high_buzz) + (((q_r_high_c1*alpha2)*theta)*q_high_pitch_c1)/2 : (s2'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s2=high_blink_med_buzz &color2=1& hw_state=0 & env_state2=1  & sound=1& Rec_motif2=true & staggered=false->((q_r_high_c1*alpha2)*theta)*q_high_pitch_c1  : (s2'=med_blink_med_buzz) + 1-(((q_r_high_c1*alpha2)*theta)*q_high_pitch_c1) : (s2'=med_blink_high_buzz);
  [step] s2=high_blink_med_buzz &color2=1& hw_state=0 & env_state2=0  & (sound=0 & move=0)& Rec_motif2=true & staggered=false -> ((q_r_high_c1*alpha2)*theta)*q_high_pitch_c1 : (s2'=finish)  + (1-(((q_r_high_c1*alpha2)*theta)*q_high_pitch_c1))/2 : (s2'=med_blink_high_buzz) + (1-(((q_r_high_c1*alpha2)*theta)*q_high_pitch_c1))/2 : (s2'=med_blink_med_buzz); //default(empirical)
 

 //med-high simultaneous actuating white
  [step] s2=med_blink_high_buzz &color2=3 & (hw_state=2 | hw_state=1) & Rec_motif2=true & staggered=false -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s2=med_blink_high_buzz &color2=3 & hw_state=0 & (env_state2=2 | env_state2=1)   & (sound=1 | move=1)& Rec_motif2=true & staggered=false-> (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s2=med_blink_high_buzz &color2=3 & hw_state=0 & env_state2=0  & (sound=0 & move=0)& Rec_motif2=true & staggered=false-> ((q_w_high_c1*alpha2)*theta)*q_high_pitch_c1*beta1 : (s2'=finish)  + 1-(((q_w_high_c1*alpha2)*theta)*q_high_pitch_c1*beta1) : (s2'=med_blink_med_buzz); //default(empirical)
  [step] s2=med_blink_high_buzz &color2=3 & hw_state=0 & env_state2=0  & ( move=1)& Rec_motif2=true & staggered=false-> 1-(((q_w_high_c1*alpha2)*theta)*q_high_pitch_c1*beta1) : (s2'=task_failed)  + ((q_w_high_c1*alpha2)*theta)*q_high_pitch_c1*beta1 : (s2'=med_blink_med_buzz); //when movement un-intentionally
 //med-high simultaneous actuating blue
  [step] s2=med_blink_high_buzz &color2=2 & (hw_state=2 | hw_state=1)& Rec_motif2=true & staggered=false  -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s2=med_blink_high_buzz &color2=2 & hw_state=0 & (env_state2=2 | env_state2=1)   & (sound=1 | move=1)& Rec_motif2=true & staggered=false-> (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s2=med_blink_high_buzz &color2=2 & hw_state=0 & env_state2=0  & (sound=0 & move=0)& Rec_motif2=true & staggered=false-> ((q_b_high_c1*alpha2)*theta)*q_high_pitch_c1*beta1 : (s2'=finish)  + 1-(((q_b_high_c1*alpha2)*theta)*q_high_pitch_c1*beta1) : (s2'=med_blink_med_buzz); //default(empirical)
  [step] s2=med_blink_high_buzz &color2=2 & hw_state=0 & env_state2=0  & ( move=1)& Rec_motif2=true & staggered=false-> 1-(((q_b_high_c1*alpha2)*theta)*q_high_pitch_c1*beta1) : (s2'=task_failed)  + ((q_b_high_c1*alpha2)*theta)*q_high_pitch_c1*beta1 : (s2'=med_blink_med_buzz); //when movement un-intentionally
 //med-high simultaneous actuating Red
  [step] s2=med_blink_high_buzz &color2=1 & (hw_state=2 | hw_state=1) & Rec_motif2=true & staggered=false -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s2=med_blink_high_buzz &color2=1 & hw_state=0 & (env_state2=2 | env_state2=1)   & (sound=1 | move=1)& Rec_motif2=true & staggered=false-> (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s2=med_blink_high_buzz &color2=1 & hw_state=0 & env_state2=0  & (sound=0 & move=0)& Rec_motif2=true & staggered=false-> ((q_r_high_c1*alpha2)*theta)*q_high_pitch_c1*beta1 : (s2'=finish)  + 1-(((q_r_high_c1*alpha2)*theta)*q_high_pitch_c1*beta1) : (s2'=med_blink_med_buzz); //default(empirical)
  [step] s2=med_blink_high_buzz &color2=1 & hw_state=0 & env_state2=0  & ( move=1) & Rec_motif2=true & staggered=false-> 1-(((q_r_high_c1*alpha2)*theta)*q_high_pitch_c1*beta1) : (s2'=task_failed)  + ((q_r_high_c1*alpha2)*theta)*q_high_pitch_c1*beta1 : (s2'=med_blink_med_buzz); //when movement un-intentionally



//med-med simultaneous actuating white
  [step] s2=med_blink_med_buzz &color2=3& (hw_state=2 |env_state2=2)& Rec_motif2=true & staggered=false -> 1-(((q_w_high_c1*alpha2)*theta)*q_high_pitch_c1*beta2) : (s2'=finish) + (((q_w_high_c1*alpha2)*theta)*q_high_pitch_c1*beta2) : (s2'=task_failed); 
  [step] s2=med_blink_med_buzz &color2=3& (hw_state=1 |env_state2=1) & Rec_motif2=true & staggered=false-> 0.5 : (s2'=finish) + 0.5 : (s2'=task_failed); 
  [step] s2=med_blink_med_buzz &color2=3& hw_state=0& Rec_motif2=true & staggered=false-> (((q_w_high_c1*alpha2)*theta)*q_high_pitch_c1*beta2) : (s2'=finish) + 1-(((q_w_high_c1*alpha2)*theta)*q_high_pitch_c1*beta2) : (s2'=task_failed);//empirical data
//med-med simultaneous actuating Blue
[step] s2=med_blink_med_buzz &color2=2& (hw_state=2 |env_state2=2) & Rec_motif2=true & staggered=false-> 1-(((q_b_high_c1*alpha2)*theta)*q_high_pitch_c1*beta2) : (s2'=finish) + (((q_b_high_c1*alpha2)*theta)*q_high_pitch_c1*beta2) : (s2'=task_failed); 
  [step] s2=med_blink_med_buzz &color2=2& (hw_state=1 |env_state2=1)& Rec_motif2=true & staggered=false -> 0.5 : (s2'=finish) + 0.5 : (s2'=task_failed); 
  [step] s2=med_blink_med_buzz &color2=2& hw_state=0& Rec_motif2=true & staggered=false-> (((q_b_high_c1*alpha2)*theta)*q_high_pitch_c1*beta2) : (s2'=finish) + 1-(((q_b_high_c1*alpha2)*theta)*q_high_pitch_c1*beta2) : (s2'=task_failed);//empirical data
//med-med simultaneous actuating Red
  [step] s2=med_blink_med_buzz &color2=1& (hw_state=2 |env_state2=2)& Rec_motif2=true & staggered=false -> 1-(((q_r_high_c1*alpha2)*theta)*q_high_pitch_c1*beta2) : (s2'=finish) + (((q_r_high_c1*alpha2)*theta)*q_high_pitch_c1*beta2) : (s2'=task_failed); 
  [step] s2=med_blink_med_buzz &color2=1& (hw_state=1 |env_state2=1) & Rec_motif2=true & staggered=false-> 0.5 : (s2'=finish) + 0.5 : (s2'=task_failed); 
  [step] s2=med_blink_med_buzz &color2=1& hw_state=0 & Rec_motif2=true & staggered=false-> (((q_r_high_c1*alpha2)*theta)*q_high_pitch_c1*beta2) : (s2'=finish) + 1-(((q_r_high_c1*alpha2)*theta)*q_high_pitch_c1*beta2) : (s2'=task_failed);//empirical data

//High LED white
  [step] s2=high_Led  &color2=3& hw_state=2 & Rec_motif2=true & staggered=false -> (s2'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s2=high_Led  &color2=3& hw_state=1 & Rec_motif2=true & staggered=false -> (s2'=med_Led);// Priority 2: hardware MED -> reduce one level
  [step] s2=high_Led  &color2=3& hw_state=0 & env_state2=2 & move=1& Rec_motif2=true & staggered=false->0.5  : (s2'=med_Led) + 0.5 : (s2'=low_Led);// Priority 3: environment sensing
  [step] s2=high_Led  &color2=3& hw_state=0 & env_state2=0 & move=1& Rec_motif2=true & staggered=false->(1-((q_w_high_c1*alpha2)*theta)) : (s2'=task_failed)  +((q_w_high_c1*alpha2)*theta)/2  : (s2'=med_Led) + ((q_w_high_c1*alpha2)*theta)/2 : (s2'=low_Led);// when movement un-intentionally
  [step] s2=high_Led  &color2=3& hw_state=0 & env_state2=1  & sound=1& Rec_motif2=true & staggered=false->0.5 : (s2'=med_Led) + 0.5 : (s2'=low_Led);
  [step] s2=high_Led  &color2=3& hw_state=0 & env_state2=0 & (sound=0 & move=0) & Rec_motif2=true & staggered=false-> ((q_w_high_c1*alpha2)*theta) : (s2'=finish) + (1-((q_w_high_c1*alpha2)*theta))/2 : (s2'=med_Led) + (1-((q_w_high_c1*alpha2)*theta))/2 : (s2'=low_Led); // default (empirical)
//High LED Blue
  [step] s2=high_Led  &color2=2& hw_state=2 & Rec_motif2=true & staggered=false -> (s2'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s2=high_Led  &color2=2& hw_state=1 & Rec_motif2=true & staggered=false -> (s2'=med_Led);// Priority 2: hardware MED -> reduce one level
  [step] s2=high_Led  &color2=2& hw_state=0 & env_state2=2 & move=1& Rec_motif2=true & staggered=false->0.5  : (s2'=med_Led) + 0.5 : (s2'=low_Led);// Priority 3: environment sensing
  [step] s2=high_Led  &color2=2& hw_state=0 & env_state2=0 & move=1& Rec_motif2=true & staggered=false->(1-((q_b_high_c1*alpha2)*theta)) : (s2'=task_failed)  +((q_b_high_c1*alpha2)*theta)/2  : (s2'=med_Led) + ((q_b_high_c1*alpha2)*theta)/2 : (s2'=low_Led);// when movement un-intentionally
  [step] s2=high_Led  &color2=2& hw_state=0 & env_state2=1  & sound=1& Rec_motif2=true & staggered=false->0.5 : (s2'=med_Led) + 0.5 : (s2'=low_Led);
  [step] s2=high_Led  &color2=2& hw_state=0 & env_state2=0 & (sound=0 & move=0)& Rec_motif2=true & staggered=false -> ((q_b_high_c1*alpha2)*theta) : (s2'=finish) + (1-((q_b_high_c1*alpha2)*theta))/2 : (s2'=med_Led) + (1-((q_b_high_c1*alpha2)*theta))/2 : (s2'=low_Led); // default (empirical)
//High LED Red
   [step] s2=high_Led  &color2=1& hw_state=2 & Rec_motif2=true & staggered=false -> (s2'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s2=high_Led  &color2=1& hw_state=1 & Rec_motif2=true & staggered=false -> (s2'=med_Led);// Priority 2: hardware MED -> reduce one level
  [step] s2=high_Led  &color2=1& hw_state=0 & env_state2=2 & move=1& Rec_motif2=true & staggered=false->0.5  : (s2'=med_Led) + 0.5 : (s2'=low_Led);// Priority 3: environment sensing
  [step] s2=high_Led  &color2=1& hw_state=0 & env_state2=0 & move=1& Rec_motif2=true & staggered=false->(1-((q_r_high_c1*alpha2)*theta)) : (s2'=task_failed)  +((q_r_high_c1*alpha2)*theta)/2  : (s2'=med_Led) + ((q_r_high_c1*alpha2)*theta)/2 : (s2'=low_Led);// when movement un-intentionally
  [step] s2=high_Led  &color2=1& hw_state=0 & env_state2=1  & sound=1& Rec_motif2=true & staggered=false->0.5 : (s2'=med_Led) + 0.5 : (s2'=low_Led);
  [step] s2=high_Led  &color2=1& hw_state=0 & env_state2=0 & (sound=0 & move=0)& Rec_motif2=true & staggered=false -> ((q_r_high_c1*alpha2)*theta) : (s2'=finish) + (1-((q_r_high_c1*alpha2)*theta))/2 : (s2'=med_Led) + (1-((q_r_high_c1*alpha2)*theta))/2 : (s2'=low_Led); // default (empirical)

//Med LED white
  [step] s2=med_Led  &color2=3& (hw_state=2 |hw_state=1) & Rec_motif2=true & staggered=false -> (s2'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [step] s2=med_Led  &color2=3& hw_state=0 & (env_state2=2 |env_state2=1)  & (sound=1 | move=1)& Rec_motif2=true & staggered=false-> (s2'=low_Led);// Priority 3: environment sensing
  [step] s2=med_Led  &color2=3& hw_state=0 & (sound=0 | move=0)& Rec_motif2=true & staggered=false-> ((q_w_high_c1*alpha2)*theta)*beta1 : (s2'=finish) + 1-(((q_w_high_c1*alpha2)*theta)*beta1) : (s2'=low_Led);// default (empirical)
  [step] s2=med_Led  &color2=3& hw_state=0 & (move=1)& Rec_motif2=true & staggered=false-> 1-(((q_w_high_c1*alpha2)*theta)*beta1) : (s2'=task_failed) + ((q_w_high_c1*alpha2)*theta)*beta1 : (s2'=low_Led);// when movement un-intentionally
//Med LED Blue
  [step] s2=med_Led   &color2=2& (hw_state=2 |hw_state=1) & Rec_motif2=true & staggered=false -> (s2'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [step] s2=med_Led  &color2=2& hw_state=0 & (env_state2=2 |env_state2=1)  & (sound=1 | move=1)& Rec_motif2=true & staggered=false-> (s2'=low_Led);// Priority 3: environment sensing
  [step] s2=med_Led  &color2=2& hw_state=0 & (sound=0 | move=0)& Rec_motif2=true & staggered=false-> ((q_b_high_c1*alpha2)*theta)*beta1 : (s2'=finish) + 1-(((q_b_high_c1*alpha2)*theta)*beta1) : (s2'=low_Led);// default (empirical)
  [step] s2=med_Led  &color2=2& hw_state=0 & (move=1)& Rec_motif2=true & staggered=false-> 1-(((q_b_high_c1*alpha2)*theta)*beta1) : (s2'=task_failed) + ((q_b_high_c1*alpha2)*theta)*beta1 : (s2'=low_Led);// when movement un-intentionally
//Med LED Red
  [step] s2=med_Led   &color2=1& (hw_state=2 |hw_state=1) & Rec_motif2=true & staggered=false -> (s2'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [step] s2=med_Led  &color2=1& hw_state=0 & (env_state2=2 |env_state2=1)  & (sound=1 | move=1)& Rec_motif2=true & staggered=false-> (s2'=low_Led);// Priority 3: environment sensing
  [step] s2=med_Led  &color2=1& hw_state=0 & (sound=0 | move=0)& Rec_motif2=true & staggered=false-> ((q_r_high_c1*alpha2)*theta)*beta1 : (s2'=finish) + 1-(((q_r_high_c1*alpha2)*theta)*beta1) : (s2'=low_Led);// default (empirical)
  [step] s2=med_Led  &color2=1& hw_state=0 & (move=1)& Rec_motif2=true & staggered=false-> 1-(((q_r_high_c1*alpha2)*theta)*beta1) : (s2'=task_failed) + ((q_r_high_c1*alpha2)*theta)*beta1 : (s2'=low_Led);// when movement un-intentionally

//low led white
  [step] s2=low_Led &color2=3& (hw_state=2 |env_state2=2) & Rec_motif2=true & staggered=false-> 1-(((q_w_high_c1*alpha2)*theta)*beta2) : (s2'=finish) + ((q_w_high_c1*alpha2)*theta)*beta2 : (s2'=task_failed); 
  [step] s2=low_Led &color2=3& (hw_state=0 |hw_state=1)& (env_state2=1 |env_state2=0)& Rec_motif2=true & staggered=false-> ((q_w_high_c1*alpha2)*theta)*beta2 : (s2'=finish) + 1-(((q_w_high_c1*alpha2)*theta)*beta2) : (s2'=task_failed);// default
//low led Blue
  [step] s2=low_Led &color2=2& (hw_state=2 |env_state2=2)& Rec_motif2=true & staggered=false -> 1-(((q_b_high_c1*alpha2)*theta)*beta2) : (s2'=finish) + ((q_b_high_c1*alpha2)*theta)*beta2 : (s2'=task_failed); 
  [step] s2=low_Led &color2=2& (hw_state=0 |hw_state=1)& (env_state2=1 |env_state2=0)& Rec_motif2=true & staggered=false-> ((q_b_high_c1*alpha2)*theta)*beta2 : (s2'=finish) + 1-(((q_b_high_c1*alpha2)*theta)*beta2) : (s2'=task_failed);// default
//low led Red
   [step] s2=low_Led &color2=1& (hw_state=2 |env_state2=2)& Rec_motif2=true & staggered=false -> 1-(((q_r_high_c1*alpha2)*theta)*beta2) : (s2'=finish) + ((q_r_high_c1*alpha2)*theta)*beta2 : (s2'=task_failed); 
  [step] s2=low_Led &color2=1& (hw_state=0 |hw_state=1)& (env_state2=1 |env_state2=0)& Rec_motif2=true & staggered=false-> ((q_r_high_c1*alpha2)*theta)*beta2 : (s2'=finish) + 1-(((q_r_high_c1*alpha2)*theta)*beta2) : (s2'=task_failed);// default
//////////////////////////////

//FINISH
 [step] s2=finish -> (s2'=finish);// absorbing states if synced with other blocks
 [step] s2=task_failed -> (s2'=task_failed); // absorbing states if synced with other blocks
  // Reset on tick
  [Task_increment] s2=finish  -> (s2'=start_task) ;
  [Task_increment] s2=task_failed  -> (s2'=start_task) ;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 // Probabilistic transitions (BB1-specific) LINE pattern staggered with other clusters
///////////////////////////////////
//high pitch
  [stag2] s2=high_pitch & hw_state=2 & Rec_motif2=false & staggered=true -> (s2'=low_pitch); // Priority 1: hardware HIGH -> immediate downgrade
  [stag2] s2=high_pitch & hw_state=1 & Rec_motif2=false & staggered=true -> (s2'=med_pitch);// Priority 2: hardware MED -> reduce one level
  [stag2] s2=high_pitch & hw_state=0 & env_state2=2  & move=1& Rec_motif2=false & staggered=true ->(1-q_high_pitch_c1)/2 : (s2'=med_pitch) + 1-((1-q_high_pitch_c1)/2) : (s2'=low_pitch);// Priority 3: environment sensing
  [stag2] s2=high_pitch & hw_state=0 & env_state2= 0 & move=1& Rec_motif2=false & staggered=true->(1-q_high_pitch_c1) : (s2'=task_failed)+ q_high_pitch_c1/2 : (s2'=med_pitch) + q_high_pitch_c1/2 : (s2'=low_pitch);// when move (no direct intention of fallback), chance of failure occurs
  [stag2] s2=high_pitch & hw_state=0 & env_state2=1  & sound=1 & Rec_motif2=false & staggered=true->1-((1-q_high_pitch_c1)/2) : (s2'=med_pitch) + (1-q_high_pitch_c1)/2 : (s2'=low_pitch);
  [stag2] s2=high_pitch& hw_state=0   & (sound=0 & move=0) & Rec_motif2=false & staggered=true-> q_high_pitch_c1  : (s2'=finish) + (1-q_high_pitch_c1)/2 : (s2'=med_pitch) + (1-q_high_pitch_c1)/2 : (s2'=low_pitch); // default

//med pitch
  [stag2] s2=med_pitch & (hw_state=2 |hw_state=1) & Rec_motif2=false & staggered=true -> (s2'=low_pitch); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [stag2] s2=med_pitch & hw_state=0 & (env_state2=2 |env_state2=1)  &( sound=1 |move=1)& Rec_motif2=false & staggered=true -> (s2'=low_pitch);// Priority 3: environment sensing
  [stag2] s2=med_pitch & hw_state=0 & env_state2= 0 & move=1& Rec_motif2=false & staggered=true-> 1-(q_high_pitch_c1*beta1) : (s2'=task_failed)+ q_high_pitch_c1*beta1 : (s2'=low_pitch);// when move (no direct intention of fallback), chance of failure occurs
  [stag2] s2=med_pitch& hw_state=0  & (sound=0 & move=0)& Rec_motif2=false & staggered=true-> q_high_pitch_c1*beta1 : (s2'=finish) + (1-(q_high_pitch_c1*beta1)) : (s2'=low_pitch);// default

//low pitch
  [stag2] s2=low_pitch & (hw_state=2 |env_state2=2)& Rec_motif2=false & staggered=true -> (1-(q_high_pitch_c1*beta2)) : (s2'=finish) + q_high_pitch_c1*beta2 : (s2'=task_failed); 
  [stag2] s2=low_pitch & (hw_state=1 |env_state2=1)& Rec_motif2=false & staggered=true -> (1-((q_high_pitch_c1*beta2)/2)) : (s2'=finish) + (q_high_pitch_c1*beta2)/2 : (s2'=task_failed); 
  [stag2] s2=low_pitch & hw_state=0 & Rec_motif2=false & staggered=true-> q_high_pitch_c1*beta2 : (s2'=finish) + (1-(q_high_pitch_c1*beta2)) : (s2'=task_failed);//even if movement it is already very low consuming and it is the lowest state so failure exist but not as when a higher consuming state

//sensor-environment transition
  [stag2] s2=listen_on_moving_on & Rec_motif2=false & staggered=true-> 0.95 : (s2'=finish) + 0.05 : (s2'=task_failed);
  [stag2] s2=listen_on_moving_off & Rec_motif2=false & staggered=true-> 0.95 : (s2'=finish) + 0.05 : (s2'=task_failed);
  [stag2] s2=listen_off_moving_on& Rec_motif2=false & staggered=true -> 0.95 : (s2'=finish) + 0.05 : (s2'=task_failed);

//high-high simultaneous actuating white
  [stag2] s2=high_blink_high_buzz &color2=3 & hw_state=2 & Rec_motif2=false & staggered=true -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag2] s2=high_blink_high_buzz &color2=3 & hw_state=1 & Rec_motif2=false & staggered=true -> 0.5 : (s2'=high_blink_med_buzz) +0.5: (s2'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level
  [stag2] s2=high_blink_high_buzz &color2=3 & hw_state=0 & env_state2=2  & move=1& Rec_motif2=false & staggered=true->0.5 : (s2'=med_blink_high_buzz) + 0.5 : (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag2] s2=high_blink_high_buzz &color2=3 & hw_state=0 & env_state2=0  & move=1& Rec_motif2=false & staggered=true->1-(((q_w_high_c1*alpha2)*gamma)*q_high_pitch_c1) : (s2'=task_failed)+(((q_w_high_c1*alpha2)*gamma)*q_high_pitch_c1)/2 : (s2'=med_blink_high_buzz) + (((q_w_high_c1*alpha2)*gamma)*q_high_pitch_c1)/2 : (s2'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [stag2] s2=high_blink_high_buzz &color2=3 & hw_state=0 & env_state2=1  & sound=1& Rec_motif2=false & staggered=true->((q_w_high_c1*alpha2)*gamma)*q_high_pitch_c1 : (s2'=high_blink_med_buzz) + 1-(((q_w_high_c1*alpha2)*gamma)*q_high_pitch_c1) : (s2'=med_blink_high_buzz);
  [stag2] s2=high_blink_high_buzz &color2=3 & hw_state=0 & env_state2=0  & (sound=0 & move=0)& Rec_motif2=false & staggered=true -> ((q_w_high_c1*alpha2)*gamma)*q_high_pitch_c1: (s2'=finish) + (1-(((q_w_high_c1*alpha2)*gamma)*q_high_pitch_c1))/3 : (s2'=high_blink_med_buzz) + (1-(((q_w_high_c1*alpha2)*gamma)*q_high_pitch_c1))/3 : (s2'=med_blink_high_buzz) + (1-(((q_w_high_c1*alpha2)*gamma)*q_high_pitch_c1))/3 : (s2'=med_blink_med_buzz); //default(Empirical)
//high-high simultaneous actuating blue
  [stag2] s2=high_blink_high_buzz &color2=2 & hw_state=2 & Rec_motif2=false & staggered=true -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag2] s2=high_blink_high_buzz &color2=2 & hw_state=1 & Rec_motif2=false & staggered=true -> 0.5 : (s2'=high_blink_med_buzz) +0.5: (s2'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level
  [stag2] s2=high_blink_high_buzz &color2=2 & hw_state=0 & env_state2=2  & move=1& Rec_motif2=false & staggered=true->0.5 : (s2'=med_blink_high_buzz) + 0.5 : (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag2] s2=high_blink_high_buzz &color2=2 & hw_state=0 & env_state2=0  & move=1& Rec_motif2=false & staggered=true->1-(((q_b_high_c1*alpha2)*gamma)*q_high_pitch_c1) : (s2'=task_failed)+(((q_b_high_c1*alpha2)*gamma)*q_high_pitch_c1)/2 : (s2'=med_blink_high_buzz) + (((q_b_high_c1*alpha2)*gamma)*q_high_pitch_c1)/2 : (s2'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [stag2] s2=high_blink_high_buzz &color2=2 & hw_state=0 & env_state2=1  & sound=1& Rec_motif2=false & staggered=true->((q_b_high_c1*alpha2)*gamma)*q_high_pitch_c1 : (s2'=high_blink_med_buzz) + 1-(((q_b_high_c1*alpha2)*gamma)*q_high_pitch_c1) : (s2'=med_blink_high_buzz);
  [stag2] s2=high_blink_high_buzz &color2=2 & hw_state=0 & env_state2=0  & (sound=0 & move=0) & Rec_motif2=false & staggered=true-> ((q_b_high_c1*alpha2)*gamma)*q_high_pitch_c1: (s2'=finish) + (1-(((q_b_high_c1*alpha2)*gamma)*q_high_pitch_c1))/3 : (s2'=high_blink_med_buzz) + (1-(((q_b_high_c1*alpha2)*gamma)*q_high_pitch_c1))/3 : (s2'=med_blink_high_buzz) + (1-(((q_b_high_c1*alpha2)*gamma)*q_high_pitch_c1))/3 : (s2'=med_blink_med_buzz); //default(Empirical)
//high-high simultaneous actuating RED
  [stag2] s2=high_blink_high_buzz &color2=1 & hw_state=2 & Rec_motif2=false & staggered=true -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag2] s2=high_blink_high_buzz &color2=1 & hw_state=1 & Rec_motif2=false & staggered=true -> 0.5 : (s2'=high_blink_med_buzz) +0.5: (s2'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level
  [stag2] s2=high_blink_high_buzz &color2=1 & hw_state=0 & env_state2=2  & move=1& Rec_motif2=false & staggered=true->0.5 : (s2'=med_blink_high_buzz) + 0.5 : (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag2] s2=high_blink_high_buzz &color2=1 & hw_state=0 & env_state2=0  & move=1& Rec_motif2=false & staggered=true->1-(((q_r_high_c1*alpha2)*gamma)*q_high_pitch_c1) : (s2'=task_failed)+(((q_r_high_c1*alpha2)*gamma)*q_high_pitch_c1)/2 : (s2'=med_blink_high_buzz) + (((q_r_high_c1*alpha2)*gamma)*q_high_pitch_c1)/2 : (s2'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [stag2] s2=high_blink_high_buzz &color2=1 & hw_state=0 & env_state2=1  & sound=1& Rec_motif2=false & staggered=true->((q_r_high_c1*alpha2)*gamma)*q_high_pitch_c1 : (s2'=high_blink_med_buzz) + 1-(((q_r_high_c1*alpha2)*gamma)*q_high_pitch_c1) : (s2'=med_blink_high_buzz);
  [stag2] s2=high_blink_high_buzz &color2=1 & hw_state=0 & env_state2=0  & (sound=0 & move=0)& Rec_motif2=false & staggered=true -> ((q_r_high_c1*alpha2)*gamma)*q_high_pitch_c1: (s2'=finish) + (1-(((q_r_high_c1*alpha2)*gamma)*q_high_pitch_c1))/3 : (s2'=high_blink_med_buzz) + (1-(((q_r_high_c1*alpha2)*gamma)*q_high_pitch_c1))/3 : (s2'=med_blink_high_buzz) + (1-(((q_r_high_c1*alpha2)*gamma)*q_high_pitch_c1))/3 : (s2'=med_blink_med_buzz); //default(Empirical)

//high-med simultaneous actuating white
  [stag2] s2=high_blink_med_buzz &color2=3 & hw_state=2& Rec_motif2=false & staggered=true  -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag2] s2=high_blink_med_buzz &color2=3 & hw_state=1 & Rec_motif2=false & staggered=true -> 0.5 : (s2'=med_blink_med_buzz) + 0.5 : (s2'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level /equal probabilities cause it deoends on user preferences
  [stag2] s2=high_blink_med_buzz &color2=3& hw_state=0 & env_state2=2  & move=1& Rec_motif2=false & staggered=true->1-(((q_w_high_c1*alpha2)*gamma)*q_high_pitch_c1) : (s2'=med_blink_high_buzz) + ((q_w_high_c1*alpha2)*gamma)*q_high_pitch_c1 : (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag2] s2=high_blink_high_buzz &color2=3 & hw_state=0 & env_state2=0  & move=1& Rec_motif2=false & staggered=true->1-(((q_w_high_c1*alpha2)*gamma)*q_high_pitch_c1) : (s2'=task_failed)+(((q_w_high_c1*alpha2)*gamma)*q_high_pitch_c1)/2 : (s2'=med_blink_high_buzz) + (((q_w_high_c1*alpha2)*gamma)*q_high_pitch_c1)/2 : (s2'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [stag2] s2=high_blink_med_buzz &color2=3& hw_state=0 & env_state2=1  & sound=1& Rec_motif2=false & staggered=true->((q_w_high_c1*alpha2)*gamma)*q_high_pitch_c1  : (s2'=med_blink_med_buzz) + 1-(((q_w_high_c1*alpha2)*gamma)*q_high_pitch_c1) : (s2'=med_blink_high_buzz);
  [stag2] s2=high_blink_med_buzz &color2=3& hw_state=0 & env_state2=0  & (sound=0 & move=0) & Rec_motif2=false & staggered=true-> ((q_w_high_c1*alpha2)*gamma)*q_high_pitch_c1 : (s2'=finish)  + (1-(((q_w_high_c1*alpha2)*gamma)*q_high_pitch_c1))/2 : (s2'=med_blink_high_buzz) + (1-(((q_w_high_c1*alpha2)*gamma)*q_high_pitch_c1))/2 : (s2'=med_blink_med_buzz); //default(empirical)
 //high-med simultaneous actuating blue
   [stag2] s2=high_blink_med_buzz &color2=2 & hw_state=2 & Rec_motif2=false & staggered=true -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag2] s2=high_blink_med_buzz &color2=2 & hw_state=1 & Rec_motif2=false & staggered=true -> 0.5 : (s2'=med_blink_med_buzz) + 0.5 : (s2'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level /equal probabilities cause it deoends on user preferences
  [stag2] s2=high_blink_med_buzz &color2=2& hw_state=0 & env_state2=2  & move=1& Rec_motif2=false & staggered=true->1-(((q_b_high_c1*alpha2)*gamma)*q_high_pitch_c1) : (s2'=med_blink_high_buzz) + ((q_b_high_c1*alpha2)*gamma)*q_high_pitch_c1 : (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag2] s2=high_blink_high_buzz &color2=2 & hw_state=0 & env_state2=0  & move=1& Rec_motif2=false & staggered=true->1-(((q_b_high_c1*alpha2)*gamma)*q_high_pitch_c1) : (s2'=task_failed)+(((q_b_high_c1*alpha2)*gamma)*q_high_pitch_c1)/2 : (s2'=med_blink_high_buzz) + (((q_b_high_c1*alpha2)*gamma)*q_high_pitch_c1)/2 : (s2'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [stag2] s2=high_blink_med_buzz &color2=2& hw_state=0 & env_state2=1  & sound=1& Rec_motif2=false & staggered=true->((q_b_high_c1*alpha2)*gamma)*q_high_pitch_c1  : (s2'=med_blink_med_buzz) + 1-(((q_b_high_c1*alpha2)*gamma)*q_high_pitch_c1) : (s2'=med_blink_high_buzz);
  [stag2] s2=high_blink_med_buzz &color2=2& hw_state=0 & env_state2=0  & (sound=0 & move=0)& Rec_motif2=false & staggered=true -> ((q_b_high_c1*alpha2)*gamma)*q_high_pitch_c1 : (s2'=finish)  + (1-(((q_b_high_c1*alpha2)*gamma)*q_high_pitch_c1))/2 : (s2'=med_blink_high_buzz) + (1-(((q_b_high_c1*alpha2)*gamma)*q_high_pitch_c1))/2 : (s2'=med_blink_med_buzz); //default(empirical)
 
//high-med simultaneous actuating Red
  [stag2] s2=high_blink_med_buzz &color2=1 & hw_state=2 & Rec_motif2=false & staggered=true -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag2] s2=high_blink_med_buzz &color2=1 & hw_state=1 & Rec_motif2=false & staggered=true -> 0.5 : (s2'=med_blink_med_buzz) + 0.5 : (s2'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level /equal probabilities cause it deoends on user preferences
  [stag2] s2=high_blink_med_buzz &color2=1& hw_state=0 & env_state2=2  & move=1& Rec_motif2=false & staggered=true->1-(((q_r_high_c1*alpha2)*gamma)*q_high_pitch_c1) : (s2'=med_blink_high_buzz) + ((q_r_high_c1*alpha2)*gamma)*q_high_pitch_c1 : (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag2] s2=high_blink_high_buzz &color2=1 & hw_state=0 & env_state2=0  & move=1& Rec_motif2=false & staggered=true->1-(((q_r_high_c1*alpha2)*gamma)*q_high_pitch_c1) : (s2'=task_failed)+(((q_r_high_c1*alpha2)*gamma)*q_high_pitch_c1)/2 : (s2'=med_blink_high_buzz) + (((q_r_high_c1*alpha2)*gamma)*q_high_pitch_c1)/2 : (s2'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [stag2] s2=high_blink_med_buzz &color2=1& hw_state=0 & env_state2=1  & sound=1& Rec_motif2=false & staggered=true->((q_r_high_c1*alpha2)*gamma)*q_high_pitch_c1  : (s2'=med_blink_med_buzz) + 1-(((q_r_high_c1*alpha2)*gamma)*q_high_pitch_c1) : (s2'=med_blink_high_buzz);
  [stag2] s2=high_blink_med_buzz &color2=1& hw_state=0 & env_state2=0  & (sound=0 & move=0)& Rec_motif2=false & staggered=true -> ((q_r_high_c1*alpha2)*gamma)*q_high_pitch_c1 : (s2'=finish)  + (1-(((q_r_high_c1*alpha2)*gamma)*q_high_pitch_c1))/2 : (s2'=med_blink_high_buzz) + (1-(((q_r_high_c1*alpha2)*gamma)*q_high_pitch_c1))/2 : (s2'=med_blink_med_buzz); //default(empirical)
 

 //med-high simultaneous actuating white
  [stag2] s2=med_blink_high_buzz &color2=3 & (hw_state=2 | hw_state=1) & Rec_motif2=false & staggered=true -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag2] s2=med_blink_high_buzz &color2=3 & hw_state=0 & (env_state2=2 | env_state2=1)   & (sound=1 | move=1)& Rec_motif2=false & staggered=true-> (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag2] s2=med_blink_high_buzz &color2=3 & hw_state=0 & env_state2=0  & (sound=0 & move=0)& Rec_motif2=false & staggered=true-> ((q_w_high_c1*alpha2)*gamma)*q_high_pitch_c1*beta1 : (s2'=finish)  + 1-(((q_w_high_c1*alpha2)*gamma)*q_high_pitch_c1*beta1) : (s2'=med_blink_med_buzz); //default(empirical)
  [stag2] s2=med_blink_high_buzz &color2=3 & hw_state=0 & env_state2=0  & ( move=1)& Rec_motif2=false & staggered=true-> 1-(((q_w_high_c1*alpha2)*gamma)*q_high_pitch_c1*beta1) : (s2'=task_failed)  + ((q_w_high_c1*alpha2)*gamma)*q_high_pitch_c1*beta1 : (s2'=med_blink_med_buzz); //when movement un-intentionally
 //med-high simultaneous actuating blue
  [stag2] s2=med_blink_high_buzz &color2=2 & (hw_state=2 | hw_state=1)& Rec_motif2=false & staggered=true  -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag2] s2=med_blink_high_buzz &color2=2 & hw_state=0 & (env_state2=2 | env_state2=1)   & (sound=1 | move=1)& Rec_motif2=false & staggered=true-> (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag2] s2=med_blink_high_buzz &color2=2 & hw_state=0 & env_state2=0  & (sound=0 & move=0)& Rec_motif2=false & staggered=true-> ((q_b_high_c1*alpha2)*gamma)*q_high_pitch_c1*beta1 : (s2'=finish)  + 1-(((q_b_high_c1*alpha2)*gamma)*q_high_pitch_c1*beta1) : (s2'=med_blink_med_buzz); //default(empirical)
  [stag2] s2=med_blink_high_buzz &color2=2 & hw_state=0 & env_state2=0  & ( move=1)& Rec_motif2=false & staggered=true-> 1-(((q_b_high_c1*alpha2)*gamma)*q_high_pitch_c1*beta1) : (s2'=task_failed)  + ((q_b_high_c1*alpha2)*gamma)*q_high_pitch_c1*beta1 : (s2'=med_blink_med_buzz); //when movement un-intentionally
 //med-high simultaneous actuating Red
  [stag2] s2=med_blink_high_buzz &color2=1 & (hw_state=2 | hw_state=1) & Rec_motif2=false & staggered=true -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag2] s2=med_blink_high_buzz &color2=1 & hw_state=0 & (env_state2=2 | env_state2=1)   & (sound=1 | move=1)& Rec_motif2=false & staggered=true-> (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag2] s2=med_blink_high_buzz &color2=1 & hw_state=0 & env_state2=0  & (sound=0 & move=0)& Rec_motif2=false & staggered=true-> ((q_r_high_c1*alpha2)*gamma)*q_high_pitch_c1*beta1 : (s2'=finish)  + 1-(((q_r_high_c1*alpha2)*gamma)*q_high_pitch_c1*beta1) : (s2'=med_blink_med_buzz); //default(empirical)
  [stag2] s2=med_blink_high_buzz &color2=1 & hw_state=0 & env_state2=0  & ( move=1) & Rec_motif2=false & staggered=true-> 1-(((q_r_high_c1*alpha2)*gamma)*q_high_pitch_c1*beta1) : (s2'=task_failed)  + ((q_r_high_c1*alpha2)*gamma)*q_high_pitch_c1*beta1 : (s2'=med_blink_med_buzz); //when movement un-intentionally



//med-med simultaneous actuating white
  [stag2] s2=med_blink_med_buzz &color2=3& (hw_state=2 |env_state2=2)& Rec_motif2=false & staggered=true -> 1-(((q_w_high_c1*alpha2)*gamma)*q_high_pitch_c1*beta2) : (s2'=finish) + (((q_w_high_c1*alpha2)*gamma)*q_high_pitch_c1*beta2) : (s2'=task_failed); 
  [stag2] s2=med_blink_med_buzz &color2=3& (hw_state=1 |env_state2=1) & Rec_motif2=false & staggered=true-> 0.5 : (s2'=finish) + 0.5 : (s2'=task_failed); 
  [stag2] s2=med_blink_med_buzz &color2=3& hw_state=0& Rec_motif2=false & staggered=true-> (((q_w_high_c1*alpha2)*gamma)*q_high_pitch_c1*beta2) : (s2'=finish) + 1-(((q_w_high_c1*alpha2)*gamma)*q_high_pitch_c1*beta2) : (s2'=task_failed);//empirical data
//med-med simultaneous actuating Blue
[stag2] s2=med_blink_med_buzz &color2=2& (hw_state=2 |env_state2=2) & Rec_motif2=false & staggered=true-> 1-(((q_b_high_c1*alpha2)*gamma)*q_high_pitch_c1*beta2) : (s2'=finish) + (((q_b_high_c1*alpha2)*gamma)*q_high_pitch_c1*beta2) : (s2'=task_failed); 
  [stag2] s2=med_blink_med_buzz &color2=2& (hw_state=1 |env_state2=1)& Rec_motif2=false & staggered=true -> 0.5 : (s2'=finish) + 0.5 : (s2'=task_failed); 
  [stag2] s2=med_blink_med_buzz &color2=2& hw_state=0& Rec_motif2=false & staggered=true-> (((q_b_high_c1*alpha2)*gamma)*q_high_pitch_c1*beta2) : (s2'=finish) + 1-(((q_b_high_c1*alpha2)*gamma)*q_high_pitch_c1*beta2) : (s2'=task_failed);//empirical data
//med-med simultaneous actuating Red
  [stag2] s2=med_blink_med_buzz &color2=1& (hw_state=2 |env_state2=2)& Rec_motif2=false & staggered=true -> 1-(((q_r_high_c1*alpha2)*gamma)*q_high_pitch_c1*beta2) : (s2'=finish) + (((q_r_high_c1*alpha2)*gamma)*q_high_pitch_c1*beta2) : (s2'=task_failed); 
  [stag2] s2=med_blink_med_buzz &color2=1& (hw_state=1 |env_state2=1) & Rec_motif2=false & staggered=true-> 0.5 : (s2'=finish) + 0.5 : (s2'=task_failed); 
  [stag2] s2=med_blink_med_buzz &color2=1& hw_state=0 & Rec_motif2=false & staggered=true-> (((q_r_high_c1*alpha2)*gamma)*q_high_pitch_c1*beta2) : (s2'=finish) + 1-(((q_r_high_c1*alpha2)*gamma)*q_high_pitch_c1*beta2) : (s2'=task_failed);//empirical data

//High LED white
  [stag2] s2=high_Led  &color2=3& hw_state=2 & Rec_motif2=false & staggered=true -> (s2'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade
  [stag2] s2=high_Led  &color2=3& hw_state=1 & Rec_motif2=false & staggered=true -> (s2'=med_Led);// Priority 2: hardware MED -> reduce one level
  [stag2] s2=high_Led  &color2=3& hw_state=0 & env_state2=2 & move=1& Rec_motif2=false & staggered=true->0.5  : (s2'=med_Led) + 0.5 : (s2'=low_Led);// Priority 3: environment sensing
  [stag2] s2=high_Led  &color2=3& hw_state=0 & env_state2=0 & move=1& Rec_motif2=false & staggered=true->(1-((q_w_high_c1*alpha2)*gamma)) : (s2'=task_failed)  +((q_w_high_c1*alpha2)*gamma)/2  : (s2'=med_Led) + ((q_w_high_c1*alpha2)*gamma)/2 : (s2'=low_Led);// when movement un-intentionally
  [stag2] s2=high_Led  &color2=3& hw_state=0 & env_state2=1  & sound=1& Rec_motif2=false & staggered=true->0.5 : (s2'=med_Led) + 0.5 : (s2'=low_Led);
  [stag2] s2=high_Led  &color2=3& hw_state=0 & env_state2=0 & (sound=0 & move=0) & Rec_motif2=false & staggered=true-> ((q_w_high_c1*alpha2)*gamma) : (s2'=finish) + (1-((q_w_high_c1*alpha2)*gamma))/2 : (s2'=med_Led) + (1-((q_w_high_c1*alpha2)*gamma))/2 : (s2'=low_Led); // default (empirical)
//High LED Blue
  [stag2] s2=high_Led  &color2=2& hw_state=2 & Rec_motif2=false & staggered=true -> (s2'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade
  [stag2] s2=high_Led  &color2=2& hw_state=1 & Rec_motif2=false & staggered=true -> (s2'=med_Led);// Priority 2: hardware MED -> reduce one level
  [stag2] s2=high_Led  &color2=2& hw_state=0 & env_state2=2 & move=1& Rec_motif2=false & staggered=true->0.5  : (s2'=med_Led) + 0.5 : (s2'=low_Led);// Priority 3: environment sensing
  [stag2] s2=high_Led  &color2=2& hw_state=0 & env_state2=0 & move=1& Rec_motif2=false & staggered=true->(1-((q_b_high_c1*alpha2)*gamma)) : (s2'=task_failed)  +((q_b_high_c1*alpha2)*gamma)/2  : (s2'=med_Led) + ((q_b_high_c1*alpha2)*gamma)/2 : (s2'=low_Led);// when movement un-intentionally
  [stag2] s2=high_Led  &color2=2& hw_state=0 & env_state2=1  & sound=1& Rec_motif2=false & staggered=true->0.5 : (s2'=med_Led) + 0.5 : (s2'=low_Led);
  [stag2] s2=high_Led  &color2=2& hw_state=0 & env_state2=0 & (sound=0 & move=0)& Rec_motif2=false & staggered=true -> ((q_b_high_c1*alpha2)*gamma) : (s2'=finish) + (1-((q_b_high_c1*alpha2)*gamma))/2 : (s2'=med_Led) + (1-((q_b_high_c1*alpha2)*gamma))/2 : (s2'=low_Led); // default (empirical)
//High LED Red
   [stag2] s2=high_Led  &color2=1& hw_state=2 & Rec_motif2=false & staggered=true -> (s2'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade
  [stag2] s2=high_Led  &color2=1& hw_state=1 & Rec_motif2=false & staggered=true -> (s2'=med_Led);// Priority 2: hardware MED -> reduce one level
  [stag2] s2=high_Led  &color2=1& hw_state=0 & env_state2=2 & move=1& Rec_motif2=false & staggered=true->0.5  : (s2'=med_Led) + 0.5 : (s2'=low_Led);// Priority 3: environment sensing
  [stag2] s2=high_Led  &color2=1& hw_state=0 & env_state2=0 & move=1& Rec_motif2=false & staggered=true->(1-((q_r_high_c1*alpha2)*gamma)) : (s2'=task_failed)  +((q_r_high_c1*alpha2)*gamma)/2  : (s2'=med_Led) + ((q_r_high_c1*alpha2)*gamma)/2 : (s2'=low_Led);// when movement un-intentionally
  [stag2] s2=high_Led  &color2=1& hw_state=0 & env_state2=1  & sound=1& Rec_motif2=false & staggered=true->0.5 : (s2'=med_Led) + 0.5 : (s2'=low_Led);
  [stag2] s2=high_Led  &color2=1& hw_state=0 & env_state2=0 & (sound=0 & move=0)& Rec_motif2=false & staggered=true -> ((q_r_high_c1*alpha2)*gamma) : (s2'=finish) + (1-((q_r_high_c1*alpha2)*gamma))/2 : (s2'=med_Led) + (1-((q_r_high_c1*alpha2)*gamma))/2 : (s2'=low_Led); // default (empirical)

//Med LED white
  [stag2] s2=med_Led  &color2=3& (hw_state=2 |hw_state=1) & Rec_motif2=false & staggered=true -> (s2'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [stag2] s2=med_Led  &color2=3& hw_state=0 & (env_state2=2 |env_state2=1)  & (sound=1 | move=1)& Rec_motif2=false & staggered=true-> (s2'=low_Led);// Priority 3: environment sensing
  [stag2] s2=med_Led  &color2=3& hw_state=0 & (sound=0 | move=0)& Rec_motif2=false & staggered=true-> ((q_w_high_c1*alpha2)*gamma)*beta1 : (s2'=finish) + 1-(((q_w_high_c1*alpha2)*gamma)*beta1) : (s2'=low_Led);// default (empirical)
  [stag2] s2=med_Led  &color2=3& hw_state=0 & (move=1)& Rec_motif2=false & staggered=true-> 1-(((q_w_high_c1*alpha2)*gamma)*beta1) : (s2'=task_failed) + ((q_w_high_c1*alpha2)*gamma)*beta1 : (s2'=low_Led);// when movement un-intentionally
//Med LED Blue
  [stag2] s2=med_Led   &color2=2& (hw_state=2 |hw_state=1) & Rec_motif2=false & staggered=true -> (s2'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [stag2] s2=med_Led  &color2=2& hw_state=0 & (env_state2=2 |env_state2=1)  & (sound=1 | move=1)& Rec_motif2=false & staggered=true-> (s2'=low_Led);// Priority 3: environment sensing
  [stag2] s2=med_Led  &color2=2& hw_state=0 & (sound=0 | move=0)& Rec_motif2=false & staggered=true-> ((q_b_high_c1*alpha2)*gamma)*beta1 : (s2'=finish) + 1-(((q_b_high_c1*alpha2)*gamma)*beta1) : (s2'=low_Led);// default (empirical)
  [stag2] s2=med_Led  &color2=2& hw_state=0 & (move=1)& Rec_motif2=false & staggered=true-> 1-(((q_b_high_c1*alpha2)*gamma)*beta1) : (s2'=task_failed) + ((q_b_high_c1*alpha2)*gamma)*beta1 : (s2'=low_Led);// when movement un-intentionally
//Med LED Red
  [stag2] s2=med_Led   &color2=1& (hw_state=2 |hw_state=1) & Rec_motif2=false & staggered=true -> (s2'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [stag2] s2=med_Led  &color2=1& hw_state=0 & (env_state2=2 |env_state2=1)  & (sound=1 | move=1)& Rec_motif2=false & staggered=true-> (s2'=low_Led);// Priority 3: environment sensing
  [stag2] s2=med_Led  &color2=1& hw_state=0 & (sound=0 | move=0)& Rec_motif2=false & staggered=true-> ((q_r_high_c1*alpha2)*gamma)*beta1 : (s2'=finish) + 1-(((q_r_high_c1*alpha2)*gamma)*beta1) : (s2'=low_Led);// default (empirical)
  [stag2] s2=med_Led  &color2=1& hw_state=0 & (move=1)& Rec_motif2=false & staggered=true-> 1-(((q_r_high_c1*alpha2)*gamma)*beta1) : (s2'=task_failed) + ((q_r_high_c1*alpha2)*gamma)*beta1 : (s2'=low_Led);// when movement un-intentionally

//low led white
  [stag2] s2=low_Led &color2=3& (hw_state=2 |env_state2=2) & Rec_motif2=false & staggered=true-> 1-(((q_w_high_c1*alpha2)*gamma)*beta2) : (s2'=finish) + ((q_w_high_c1*alpha2)*gamma)*beta2 : (s2'=task_failed); 
  [stag2] s2=low_Led &color2=3& (hw_state=0 |hw_state=1)& (env_state2=1 |env_state2=0)& Rec_motif2=false & staggered=true-> ((q_w_high_c1*alpha2)*gamma)*beta2 : (s2'=finish) + 1-(((q_w_high_c1*alpha2)*gamma)*beta2) : (s2'=task_failed);// default
//low led Blue
  [stag2] s2=low_Led &color2=2& (hw_state=2 |env_state2=2)& Rec_motif2=false & staggered=true -> 1-(((q_b_high_c1*alpha2)*gamma)*beta2) : (s2'=finish) + ((q_b_high_c1*alpha2)*gamma)*beta2 : (s2'=task_failed); 
  [stag2] s2=low_Led &color2=2& (hw_state=0 |hw_state=1)& (env_state2=1 |env_state2=0)& Rec_motif2=false & staggered=true-> ((q_b_high_c1*alpha2)*gamma)*beta2 : (s2'=finish) + 1-(((q_b_high_c1*alpha2)*gamma)*beta2) : (s2'=task_failed);// default
//low led Red
   [stag2] s2=low_Led &color2=1& (hw_state=2 |env_state2=2)& Rec_motif2=false & staggered=true -> 1-(((q_r_high_c1*alpha2)*gamma)*beta2) : (s2'=finish) + ((q_r_high_c1*alpha2)*gamma)*beta2 : (s2'=task_failed); 
  [stag2] s2=low_Led &color2=1& (hw_state=0 |hw_state=1)& (env_state2=1 |env_state2=0)& Rec_motif2=false & staggered=true-> ((q_r_high_c1*alpha2)*gamma)*beta2 : (s2'=finish) + 1-(((q_r_high_c1*alpha2)*gamma)*beta2) : (s2'=task_failed);// default
//////////////////////////////
  // Probabilistic transitions (BB1-specific)RECTANGLE pattern staggered with other clusters
///////////////////////////////
 //high pitch
  [stag2] s2=high_pitch & hw_state=2 & Rec_motif2=true & staggered=true -> (s2'=low_pitch); // Priority 1: hardware HIGH -> immediate downgrade
  [stag2] s2=high_pitch & hw_state=1 & Rec_motif2=true & staggered=true -> (s2'=med_pitch);// Priority 2: hardware MED -> reduce one level
  [stag2] s2=high_pitch & hw_state=0 & env_state2=2  & move=1& Rec_motif2=true & staggered=true ->(1-q_high_pitch_c1)/2 : (s2'=med_pitch) + 1-((1-q_high_pitch_c1)/2) : (s2'=low_pitch);// Priority 3: environment sensing
  [stag2] s2=high_pitch & hw_state=0 & env_state2= 0 & move=1& Rec_motif2=true & staggered=true->(1-q_high_pitch_c1) : (s2'=task_failed)+ q_high_pitch_c1/2 : (s2'=med_pitch) + q_high_pitch_c1/2 : (s2'=low_pitch);// when move (no direct intention of fallback), chance of failure occurs
  [stag2] s2=high_pitch & hw_state=0 & env_state2=1  & sound=1 & Rec_motif2=true & staggered=true->1-((1-q_high_pitch_c1)/2) : (s2'=med_pitch) + (1-q_high_pitch_c1)/2 : (s2'=low_pitch);
  [stag2] s2=high_pitch& hw_state=0   & (sound=0 & move=0) & Rec_motif2=true & staggered=true-> q_high_pitch_c1  : (s2'=finish) + (1-q_high_pitch_c1)/2 : (s2'=med_pitch) + (1-q_high_pitch_c1)/2 : (s2'=low_pitch); // default

//med pitch
  [stag2] s2=med_pitch & (hw_state=2 |hw_state=1) & Rec_motif2=true & staggered=true -> (s2'=low_pitch); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [stag2] s2=med_pitch & hw_state=0 & (env_state2=2 |env_state2=1)  &( sound=1 |move=1)& Rec_motif2=true & staggered=true -> (s2'=low_pitch);// Priority 3: environment sensing
  [stag2] s2=med_pitch & hw_state=0 & env_state2= 0 & move=1& Rec_motif2=true & staggered=true-> 1-(q_high_pitch_c1*beta1) : (s2'=task_failed)+ q_high_pitch_c1*beta1 : (s2'=low_pitch);// when move (no direct intention of fallback), chance of failure occurs
  [stag2] s2=med_pitch& hw_state=0  & (sound=0 & move=0)& Rec_motif2=true & staggered=true-> q_high_pitch_c1*beta1 : (s2'=finish) + (1-(q_high_pitch_c1*beta1)) : (s2'=low_pitch);// default

//low pitch
  [stag2] s2=low_pitch & (hw_state=2 |env_state2=2)& Rec_motif2=true & staggered=true -> (1-(q_high_pitch_c1*beta2)) : (s2'=finish) + q_high_pitch_c1*beta2 : (s2'=task_failed); 
  [stag2] s2=low_pitch & (hw_state=1 |env_state2=1)& Rec_motif2=true & staggered=true -> (1-((q_high_pitch_c1*beta2)/2)) : (s2'=finish) + (q_high_pitch_c1*beta2)/2 : (s2'=task_failed); 
  [stag2] s2=low_pitch & hw_state=0 & Rec_motif2=true & staggered=true-> q_high_pitch_c1*beta2 : (s2'=finish) + (1-(q_high_pitch_c1*beta2)) : (s2'=task_failed);//even if movement it is already very low consuming and it is the lowest state so failure exist but not as when a higher consuming state

//sensor-environment transition
  [stag2] s2=listen_on_moving_on & Rec_motif2=true & staggered=true-> 0.95 : (s2'=finish) + 0.05 : (s2'=task_failed);
  [stag2] s2=listen_on_moving_off & Rec_motif2=true & staggered=true-> 0.95 : (s2'=finish) + 0.05 : (s2'=task_failed);
  [stag2] s2=listen_off_moving_on& Rec_motif2=true & staggered=true -> 0.95 : (s2'=finish) + 0.05 : (s2'=task_failed);

//high-high simultaneous actuating white
  [stag2] s2=high_blink_high_buzz &color2=3 & hw_state=2 & Rec_motif2=true & staggered=true -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag2] s2=high_blink_high_buzz &color2=3 & hw_state=1 & Rec_motif2=true & staggered=true -> 0.5 : (s2'=high_blink_med_buzz) +0.5: (s2'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level
  [stag2] s2=high_blink_high_buzz &color2=3 & hw_state=0 & env_state2=2  & move=1& Rec_motif2=true & staggered=true->0.5 : (s2'=med_blink_high_buzz) + 0.5 : (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag2] s2=high_blink_high_buzz &color2=3 & hw_state=0 & env_state2=0  & move=1& Rec_motif2=true & staggered=true->1-((((q_w_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1) : (s2'=task_failed)+((((q_w_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1)/2 : (s2'=med_blink_high_buzz) + ((((q_w_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1)/2 : (s2'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [stag2] s2=high_blink_high_buzz &color2=3 & hw_state=0 & env_state2=1  & sound=1& Rec_motif2=true & staggered=true->(((q_w_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1 : (s2'=high_blink_med_buzz) + 1-((((q_w_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1) : (s2'=med_blink_high_buzz);
  [stag2] s2=high_blink_high_buzz &color2=3 & hw_state=0 & env_state2=0  & (sound=0 & move=0)& Rec_motif2=true & staggered=true -> (((q_w_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1: (s2'=finish) + (1-((((q_w_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1))/3 : (s2'=high_blink_med_buzz) + (1-((((q_w_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1))/3 : (s2'=med_blink_high_buzz) + (1-((((q_w_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1))/3 : (s2'=med_blink_med_buzz); //default(Empirical)
//high-high simultaneous actuating blue
  [stag2] s2=high_blink_high_buzz &color2=2 & hw_state=2 & Rec_motif2=true & staggered=true -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag2] s2=high_blink_high_buzz &color2=2 & hw_state=1 & Rec_motif2=true & staggered=true -> 0.5 : (s2'=high_blink_med_buzz) +0.5: (s2'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level
  [stag2] s2=high_blink_high_buzz &color2=2 & hw_state=0 & env_state2=2  & move=1& Rec_motif2=true & staggered=true->0.5 : (s2'=med_blink_high_buzz) + 0.5 : (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag2] s2=high_blink_high_buzz &color2=2 & hw_state=0 & env_state2=0  & move=1& Rec_motif2=true & staggered=true->1-((((q_b_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1) : (s2'=task_failed)+((((q_b_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1)/2 : (s2'=med_blink_high_buzz) + ((((q_b_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1)/2 : (s2'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [stag2] s2=high_blink_high_buzz &color2=2 & hw_state=0 & env_state2=1  & sound=1& Rec_motif2=true & staggered=true->(((q_b_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1 : (s2'=high_blink_med_buzz) + 1-((((q_b_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1) : (s2'=med_blink_high_buzz);
  [stag2] s2=high_blink_high_buzz &color2=2 & hw_state=0 & env_state2=0  & (sound=0 & move=0) & Rec_motif2=true & staggered=true-> (((q_b_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1: (s2'=finish) + (1-((((q_b_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1))/3 : (s2'=high_blink_med_buzz) + (1-((((q_b_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1))/3 : (s2'=med_blink_high_buzz) + (1-((((q_b_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1))/3 : (s2'=med_blink_med_buzz); //default(Empirical)
//high-high simultaneous actuating RED
  [stag2] s2=high_blink_high_buzz &color2=1 & hw_state=2 & Rec_motif2=true & staggered=true -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag2] s2=high_blink_high_buzz &color2=1 & hw_state=1 & Rec_motif2=true & staggered=true -> 0.5 : (s2'=high_blink_med_buzz) +0.5: (s2'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level
  [stag2] s2=high_blink_high_buzz &color2=1 & hw_state=0 & env_state2=2  & move=1& Rec_motif2=true & staggered=true->0.5 : (s2'=med_blink_high_buzz) + 0.5 : (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag2] s2=high_blink_high_buzz &color2=1 & hw_state=0 & env_state2=0  & move=1& Rec_motif2=true & staggered=true->1-((((q_r_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1) : (s2'=task_failed)+((((q_r_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1)/2 : (s2'=med_blink_high_buzz) + ((((q_r_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1)/2 : (s2'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [stag2] s2=high_blink_high_buzz &color2=1 & hw_state=0 & env_state2=1  & sound=1& Rec_motif2=true & staggered=true->(((q_r_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1 : (s2'=high_blink_med_buzz) + 1-((((q_r_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1) : (s2'=med_blink_high_buzz);
  [stag2] s2=high_blink_high_buzz &color2=1 & hw_state=0 & env_state2=0  & (sound=0 & move=0)& Rec_motif2=true & staggered=true -> (((q_r_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1: (s2'=finish) + (1-((((q_r_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1))/3 : (s2'=high_blink_med_buzz) + (1-((((q_r_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1))/3 : (s2'=med_blink_high_buzz) + (1-((((q_r_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1))/3 : (s2'=med_blink_med_buzz); //default(Empirical)

//high-med simultaneous actuating white
  [stag2] s2=high_blink_med_buzz &color2=3 & hw_state=2& Rec_motif2=true & staggered=true  -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag2] s2=high_blink_med_buzz &color2=3 & hw_state=1 & Rec_motif2=true & staggered=true -> 0.5 : (s2'=med_blink_med_buzz) + 0.5 : (s2'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level /equal probabilities cause it deoends on user preferences
  [stag2] s2=high_blink_med_buzz &color2=3& hw_state=0 & env_state2=2  & move=1& Rec_motif2=true & staggered=true->1-((((q_w_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1) : (s2'=med_blink_high_buzz) + (((q_w_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1 : (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag2] s2=high_blink_high_buzz &color2=3 & hw_state=0 & env_state2=0  & move=1& Rec_motif2=true & staggered=true->1-((((q_w_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1) : (s2'=task_failed)+((((q_w_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1)/2 : (s2'=med_blink_high_buzz) + ((((q_w_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1)/2 : (s2'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [stag2] s2=high_blink_med_buzz &color2=3& hw_state=0 & env_state2=1  & sound=1& Rec_motif2=true & staggered=true->(((q_w_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1  : (s2'=med_blink_med_buzz) + 1-((((q_w_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1) : (s2'=med_blink_high_buzz);
  [stag2] s2=high_blink_med_buzz &color2=3& hw_state=0 & env_state2=0  & (sound=0 & move=0) & Rec_motif2=true & staggered=true-> (((q_w_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1 : (s2'=finish)  + (1-((((q_w_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1))/2 : (s2'=med_blink_high_buzz) + (1-((((q_w_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1))/2 : (s2'=med_blink_med_buzz); //default(empirical)
 //high-med simultaneous actuating blue
   [stag2] s2=high_blink_med_buzz &color2=2 & hw_state=2 & Rec_motif2=true & staggered=true -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag2] s2=high_blink_med_buzz &color2=2 & hw_state=1 & Rec_motif2=true & staggered=true -> 0.5 : (s2'=med_blink_med_buzz) + 0.5 : (s2'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level /equal probabilities cause it deoends on user preferences
  [stag2] s2=high_blink_med_buzz &color2=2& hw_state=0 & env_state2=2  & move=1& Rec_motif2=true & staggered=true->1-((((q_b_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1) : (s2'=med_blink_high_buzz) + (((q_b_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1 : (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag2] s2=high_blink_high_buzz &color2=2 & hw_state=0 & env_state2=0  & move=1& Rec_motif2=true & staggered=true->1-((((q_b_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1) : (s2'=task_failed)+((((q_b_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1)/2 : (s2'=med_blink_high_buzz) + ((((q_b_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1)/2 : (s2'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [stag2] s2=high_blink_med_buzz &color2=2& hw_state=0 & env_state2=1  & sound=1& Rec_motif2=true & staggered=true->(((q_b_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1  : (s2'=med_blink_med_buzz) + 1-((((q_b_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1) : (s2'=med_blink_high_buzz);
  [stag2] s2=high_blink_med_buzz &color2=2& hw_state=0 & env_state2=0  & (sound=0 & move=0)& Rec_motif2=true & staggered=true -> (((q_b_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1 : (s2'=finish)  + (1-((((q_b_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1))/2 : (s2'=med_blink_high_buzz) + (1-((((q_b_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1))/2 : (s2'=med_blink_med_buzz); //default(empirical)
 
//high-med simultaneous actuating Red
  [stag2] s2=high_blink_med_buzz &color2=1 & hw_state=2 & Rec_motif2=true & staggered=true -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag2] s2=high_blink_med_buzz &color2=1 & hw_state=1 & Rec_motif2=true & staggered=true -> 0.5 : (s2'=med_blink_med_buzz) + 0.5 : (s2'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level /equal probabilities cause it deoends on user preferences
  [stag2] s2=high_blink_med_buzz &color2=1& hw_state=0 & env_state2=2  & move=1& Rec_motif2=true & staggered=true->1-((((q_r_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1) : (s2'=med_blink_high_buzz) + (((q_r_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1 : (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag2] s2=high_blink_high_buzz &color2=1 & hw_state=0 & env_state2=0  & move=1& Rec_motif2=true & staggered=true->1-((((q_r_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1) : (s2'=task_failed)+((((q_r_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1)/2 : (s2'=med_blink_high_buzz) + ((((q_r_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1)/2 : (s2'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [stag2] s2=high_blink_med_buzz &color2=1& hw_state=0 & env_state2=1  & sound=1& Rec_motif2=true & staggered=true->(((q_r_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1  : (s2'=med_blink_med_buzz) + 1-((((q_r_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1) : (s2'=med_blink_high_buzz);
  [stag2] s2=high_blink_med_buzz &color2=1& hw_state=0 & env_state2=0  & (sound=0 & move=0)& Rec_motif2=true & staggered=true -> (((q_r_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1 : (s2'=finish)  + (1-((((q_r_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1))/2 : (s2'=med_blink_high_buzz) + (1-((((q_r_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1))/2 : (s2'=med_blink_med_buzz); //default(empirical)
 

 //med-high simultaneous actuating white
  [stag2] s2=med_blink_high_buzz &color2=3 & (hw_state=2 | hw_state=1) & Rec_motif2=true & staggered=true -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag2] s2=med_blink_high_buzz &color2=3 & hw_state=0 & (env_state2=2 | env_state2=1)   & (sound=1 | move=1)& Rec_motif2=true & staggered=true-> (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag2] s2=med_blink_high_buzz &color2=3 & hw_state=0 & env_state2=0  & (sound=0 & move=0)& Rec_motif2=true & staggered=true-> (((q_w_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1*beta1 : (s2'=finish)  + 1-((((q_w_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1*beta1) : (s2'=med_blink_med_buzz); //default(empirical)
  [stag2] s2=med_blink_high_buzz &color2=3 & hw_state=0 & env_state2=0  & ( move=1)& Rec_motif2=true & staggered=true-> 1-((((q_w_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1*beta1) : (s2'=task_failed)  + (((q_w_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1*beta1 : (s2'=med_blink_med_buzz); //when movement un-intentionally
 //med-high simultaneous actuating blue
  [stag2] s2=med_blink_high_buzz &color2=2 & (hw_state=2 | hw_state=1)& Rec_motif2=true & staggered=true  -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag2] s2=med_blink_high_buzz &color2=2 & hw_state=0 & (env_state2=2 | env_state2=1)   & (sound=1 | move=1)& Rec_motif2=true & staggered=true-> (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag2] s2=med_blink_high_buzz &color2=2 & hw_state=0 & env_state2=0  & (sound=0 & move=0)& Rec_motif2=true & staggered=true-> (((q_b_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1*beta1 : (s2'=finish)  + 1-((((q_b_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1*beta1) : (s2'=med_blink_med_buzz); //default(empirical)
  [stag2] s2=med_blink_high_buzz &color2=2 & hw_state=0 & env_state2=0  & ( move=1)& Rec_motif2=true & staggered=true-> 1-((((q_b_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1*beta1) : (s2'=task_failed)  + (((q_b_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1*beta1 : (s2'=med_blink_med_buzz); //when movement un-intentionally
 //med-high simultaneous actuating Red
  [stag2] s2=med_blink_high_buzz &color2=1 & (hw_state=2 | hw_state=1) & Rec_motif2=true & staggered=true -> (s2'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [stag2] s2=med_blink_high_buzz &color2=1 & hw_state=0 & (env_state2=2 | env_state2=1)   & (sound=1 | move=1)& Rec_motif2=true & staggered=true-> (s2'=med_blink_med_buzz);// Priority 3: environment sensing
  [stag2] s2=med_blink_high_buzz &color2=1 & hw_state=0 & env_state2=0  & (sound=0 & move=0)& Rec_motif2=true & staggered=true-> (((q_r_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1*beta1 : (s2'=finish)  + 1-((((q_r_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1*beta1) : (s2'=med_blink_med_buzz); //default(empirical)
  [stag2] s2=med_blink_high_buzz &color2=1 & hw_state=0 & env_state2=0  & ( move=1) & Rec_motif2=true & staggered=true-> 1-((((q_r_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1*beta1) : (s2'=task_failed)  + (((q_r_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1*beta1 : (s2'=med_blink_med_buzz); //when movement un-intentionally



//med-med simultaneous actuating white
  [stag2] s2=med_blink_med_buzz &color2=3& (hw_state=2 |env_state2=2)& Rec_motif2=true & staggered=true -> 1-((((q_w_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1*beta2) : (s2'=finish) + ((((q_w_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1*beta2) : (s2'=task_failed); 
  [stag2] s2=med_blink_med_buzz &color2=3& (hw_state=1 |env_state2=1) & Rec_motif2=true & staggered=true-> 0.5 : (s2'=finish) + 0.5 : (s2'=task_failed); 
  [stag2] s2=med_blink_med_buzz &color2=3& hw_state=0& Rec_motif2=true & staggered=true-> ((((q_w_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1*beta2) : (s2'=finish) + 1-((((q_w_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1*beta2) : (s2'=task_failed);//empirical data
//med-med simultaneous actuating Blue
[stag2] s2=med_blink_med_buzz &color2=2& (hw_state=2 |env_state2=2) & Rec_motif2=true & staggered=true-> 1-((((q_b_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1*beta2) : (s2'=finish) + ((((q_b_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1*beta2) : (s2'=task_failed); 
  [stag2] s2=med_blink_med_buzz &color2=2& (hw_state=1 |env_state2=1)& Rec_motif2=true & staggered=true -> 0.5 : (s2'=finish) + 0.5 : (s2'=task_failed); 
  [stag2] s2=med_blink_med_buzz &color2=2& hw_state=0& Rec_motif2=true & staggered=true-> ((((q_b_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1*beta2) : (s2'=finish) + 1-((((q_b_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1*beta2) : (s2'=task_failed);//empirical data
//med-med simultaneous actuating Red
  [stag2] s2=med_blink_med_buzz &color2=1& (hw_state=2 |env_state2=2)& Rec_motif2=true & staggered=true -> 1-((((q_r_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1*beta2) : (s2'=finish) + ((((q_r_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1*beta2) : (s2'=task_failed); 
  [stag2] s2=med_blink_med_buzz &color2=1& (hw_state=1 |env_state2=1) & Rec_motif2=true & staggered=true-> 0.5 : (s2'=finish) + 0.5 : (s2'=task_failed); 
  [stag2] s2=med_blink_med_buzz &color2=1& hw_state=0 & Rec_motif2=true & staggered=true-> ((((q_r_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1*beta2) : (s2'=finish) + 1-((((q_r_high_c1*alpha2)*gamma)*theta)*q_high_pitch_c1*beta2) : (s2'=task_failed);//empirical data

//High LED white
  [stag2] s2=high_Led  &color2=3& hw_state=2 & Rec_motif2=true & staggered=true -> (s2'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade
  [stag2] s2=high_Led  &color2=3& hw_state=1 & Rec_motif2=true & staggered=true -> (s2'=med_Led);// Priority 2: hardware MED -> reduce one level
  [stag2] s2=high_Led  &color2=3& hw_state=0 & env_state2=2 & move=1& Rec_motif2=true & staggered=true->0.5  : (s2'=med_Led) + 0.5 : (s2'=low_Led);// Priority 3: environment sensing
  [stag2] s2=high_Led  &color2=3& hw_state=0 & env_state2=0 & move=1& Rec_motif2=true & staggered=true->(1-(((q_w_high_c1*alpha2)*gamma)*theta)) : (s2'=task_failed)  +(((q_w_high_c1*alpha2)*gamma)*theta)/2  : (s2'=med_Led) + (((q_w_high_c1*alpha2)*gamma)*theta)/2 : (s2'=low_Led);// when movement un-intentionally
  [stag2] s2=high_Led  &color2=3& hw_state=0 & env_state2=1  & sound=1& Rec_motif2=true & staggered=true->0.5 : (s2'=med_Led) + 0.5 : (s2'=low_Led);
  [stag2] s2=high_Led  &color2=3& hw_state=0 & env_state2=0 & (sound=0 & move=0) & Rec_motif2=true & staggered=true-> (((q_w_high_c1*alpha2)*gamma)*theta) : (s2'=finish) + (1-(((q_w_high_c1*alpha2)*gamma)*theta))/2 : (s2'=med_Led) + (1-(((q_w_high_c1*alpha2)*gamma)*theta))/2 : (s2'=low_Led); // default (empirical)
//High LED Blue
  [stag2] s2=high_Led  &color2=2& hw_state=2 & Rec_motif2=true & staggered=true -> (s2'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade
  [stag2] s2=high_Led  &color2=2& hw_state=1 & Rec_motif2=true & staggered=true -> (s2'=med_Led);// Priority 2: hardware MED -> reduce one level
  [stag2] s2=high_Led  &color2=2& hw_state=0 & env_state2=2 & move=1& Rec_motif2=true & staggered=true->0.5  : (s2'=med_Led) + 0.5 : (s2'=low_Led);// Priority 3: environment sensing
  [stag2] s2=high_Led  &color2=2& hw_state=0 & env_state2=0 & move=1& Rec_motif2=true & staggered=true->(1-(((q_b_high_c1*alpha2)*gamma)*theta)) : (s2'=task_failed)  +(((q_b_high_c1*alpha2)*gamma)*theta)/2  : (s2'=med_Led) + (((q_b_high_c1*alpha2)*gamma)*theta)/2 : (s2'=low_Led);// when movement un-intentionally
  [stag2] s2=high_Led  &color2=2& hw_state=0 & env_state2=1  & sound=1& Rec_motif2=true & staggered=true->0.5 : (s2'=med_Led) + 0.5 : (s2'=low_Led);
  [stag2] s2=high_Led  &color2=2& hw_state=0 & env_state2=0 & (sound=0 & move=0)& Rec_motif2=true & staggered=true -> (((q_b_high_c1*alpha2)*gamma)*theta) : (s2'=finish) + (1-(((q_b_high_c1*alpha2)*gamma)*theta))/2 : (s2'=med_Led) + (1-(((q_b_high_c1*alpha2)*gamma)*theta))/2 : (s2'=low_Led); // default (empirical)
//High LED Red
   [stag2] s2=high_Led  &color2=1& hw_state=2 & Rec_motif2=true & staggered=true -> (s2'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade
  [stag2] s2=high_Led  &color2=1& hw_state=1 & Rec_motif2=true & staggered=true -> (s2'=med_Led);// Priority 2: hardware MED -> reduce one level
  [stag2] s2=high_Led  &color2=1& hw_state=0 & env_state2=2 & move=1& Rec_motif2=true & staggered=true->0.5  : (s2'=med_Led) + 0.5 : (s2'=low_Led);// Priority 3: environment sensing
  [stag2] s2=high_Led  &color2=1& hw_state=0 & env_state2=0 & move=1& Rec_motif2=true & staggered=true->(1-(((q_r_high_c1*alpha2)*gamma)*theta)) : (s2'=task_failed)  +(((q_r_high_c1*alpha2)*gamma)*theta)/2  : (s2'=med_Led) + (((q_r_high_c1*alpha2)*gamma)*theta)/2 : (s2'=low_Led);// when movement un-intentionally
  [stag2] s2=high_Led  &color2=1& hw_state=0 & env_state2=1  & sound=1& Rec_motif2=true & staggered=true->0.5 : (s2'=med_Led) + 0.5 : (s2'=low_Led);
  [stag2] s2=high_Led  &color2=1& hw_state=0 & env_state2=0 & (sound=0 & move=0)& Rec_motif2=true & staggered=true -> (((q_r_high_c1*alpha2)*gamma)*theta) : (s2'=finish) + (1-(((q_r_high_c1*alpha2)*gamma)*theta))/2 : (s2'=med_Led) + (1-(((q_r_high_c1*alpha2)*gamma)*theta))/2 : (s2'=low_Led); // default (empirical)

//Med LED white
  [stag2] s2=med_Led  &color2=3& (hw_state=2 |hw_state=1) & Rec_motif2=true & staggered=true -> (s2'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [stag2] s2=med_Led  &color2=3& hw_state=0 & (env_state2=2 |env_state2=1)  & (sound=1 | move=1)& Rec_motif2=true & staggered=true-> (s2'=low_Led);// Priority 3: environment sensing
  [stag2] s2=med_Led  &color2=3& hw_state=0 & (sound=0 | move=0)& Rec_motif2=true & staggered=true-> (((q_w_high_c1*alpha2)*gamma)*theta)*beta1 : (s2'=finish) + 1-((((q_w_high_c1*alpha2)*gamma)*theta)*beta1) : (s2'=low_Led);// default (empirical)
  [stag2] s2=med_Led  &color2=3& hw_state=0 & (move=1)& Rec_motif2=true & staggered=true-> 1-((((q_w_high_c1*alpha2)*gamma)*theta)*beta1) : (s2'=task_failed) + (((q_w_high_c1*alpha2)*gamma)*theta)*beta1 : (s2'=low_Led);// when movement un-intentionally
//Med LED Blue
  [stag2] s2=med_Led   &color2=2& (hw_state=2 |hw_state=1) & Rec_motif2=true & staggered=true -> (s2'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [stag2] s2=med_Led  &color2=2& hw_state=0 & (env_state2=2 |env_state2=1)  & (sound=1 | move=1)& Rec_motif2=true & staggered=true-> (s2'=low_Led);// Priority 3: environment sensing
  [stag2] s2=med_Led  &color2=2& hw_state=0 & (sound=0 | move=0)& Rec_motif2=true & staggered=true-> (((q_b_high_c1*alpha2)*gamma)*theta)*beta1 : (s2'=finish) + 1-((((q_b_high_c1*alpha2)*gamma)*theta)*beta1) : (s2'=low_Led);// default (empirical)
  [stag2] s2=med_Led  &color2=2& hw_state=0 & (move=1)& Rec_motif2=true & staggered=true-> 1-((((q_b_high_c1*alpha2)*gamma)*theta)*beta1) : (s2'=task_failed) + (((q_b_high_c1*alpha2)*gamma)*theta)*beta1 : (s2'=low_Led);// when movement un-intentionally
//Med LED Red
  [stag2] s2=med_Led   &color2=1& (hw_state=2 |hw_state=1) & Rec_motif2=true & staggered=true -> (s2'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [stag2] s2=med_Led  &color2=1& hw_state=0 & (env_state2=2 |env_state2=1)  & (sound=1 | move=1)& Rec_motif2=true & staggered=true-> (s2'=low_Led);// Priority 3: environment sensing
  [stag2] s2=med_Led  &color2=1& hw_state=0 & (sound=0 | move=0)& Rec_motif2=true & staggered=true-> (((q_r_high_c1*alpha2)*gamma)*theta)*beta1 : (s2'=finish) + 1-((((q_r_high_c1*alpha2)*gamma)*theta)*beta1) : (s2'=low_Led);// default (empirical)
  [stag2] s2=med_Led  &color2=1& hw_state=0 & (move=1)& Rec_motif2=true & staggered=true-> 1-((((q_r_high_c1*alpha2)*gamma)*theta)*beta1) : (s2'=task_failed) + (((q_r_high_c1*alpha2)*gamma)*theta)*beta1 : (s2'=low_Led);// when movement un-intentionally

//low led white
  [stag2] s2=low_Led &color2=3& (hw_state=2 |env_state2=2) & Rec_motif2=true & staggered=true-> 1-((((q_w_high_c1*alpha2)*gamma)*theta)*beta2) : (s2'=finish) + (((q_w_high_c1*alpha2)*gamma)*theta)*beta2 : (s2'=task_failed); 
  [stag2] s2=low_Led &color2=3& (hw_state=0 |hw_state=1)& (env_state2=1 |env_state2=0)& Rec_motif2=true & staggered=true-> (((q_w_high_c1*alpha2)*gamma)*theta)*beta2 : (s2'=finish) + 1-((((q_w_high_c1*alpha2)*gamma)*theta)*beta2) : (s2'=task_failed);// default
//low led Blue
  [stag2] s2=low_Led &color2=2& (hw_state=2 |env_state2=2)& Rec_motif2=true & staggered=true -> 1-((((q_b_high_c1*alpha2)*gamma)*theta)*beta2) : (s2'=finish) + (((q_b_high_c1*alpha2)*gamma)*theta)*beta2 : (s2'=task_failed); 
  [stag2] s2=low_Led &color2=2& (hw_state=0 |hw_state=1)& (env_state2=1 |env_state2=0)& Rec_motif2=true & staggered=true-> (((q_b_high_c1*alpha2)*gamma)*theta)*beta2 : (s2'=finish) + 1-((((q_b_high_c1*alpha2)*gamma)*theta)*beta2) : (s2'=task_failed);// default
//low led Red
   [stag2] s2=low_Led &color2=1& (hw_state=2 |env_state2=2)& Rec_motif2=true & staggered=true -> 1-((((q_r_high_c1*alpha2)*gamma)*theta)*beta2) : (s2'=finish) + (((q_r_high_c1*alpha2)*gamma)*theta)*beta2 : (s2'=task_failed); 
  [stag2] s2=low_Led &color2=1& (hw_state=0 |hw_state=1)& (env_state2=1 |env_state2=0)& Rec_motif2=true & staggered=true-> (((q_r_high_c1*alpha2)*gamma)*theta)*beta2 : (s2'=finish) + 1-((((q_r_high_c1*alpha2)*gamma)*theta)*beta2) : (s2'=task_failed);// default
//////////////////////////////
//finish states 
  [Task_increment] s2=finish & turn=1 -> (s2'=start_task) ;
  [Task_increment] s2=task_failed & turn=1  -> (s2'=start_task) ;/// Not sure if i need to put the guard of turn=1 in all state transition when staggered=true?? Going to check this !!
endmodule
// Energy rewards (scaled by cluster size)
rewards "energy"
  s1=high_pitch : 2*N1;
  s1=med_pitch : 1*N1;
  s1=low_pitch : 0*N1;
  (s1=high_blink_high_buzz & color1=1) : 8*N1;
  (s1=high_blink_high_buzz & color1=2) : 10*N1;
  (s1=high_blink_high_buzz & color1=3) : 14*N1;
  (s1=high_blink_med_buzz & color1=1) : 7*N1;
  (s1=high_blink_med_buzz & color1=2) : 9*N1;
  (s1=high_blink_med_buzz & color1=3) : 13*N1;
  (s1=med_blink_high_buzz & color1=1) : 5*N1;
  (s1=med_blink_high_buzz & color1=2) : 6*N1;
  (s1=med_blink_high_buzz & color1=3) : 8*N1;
  (s1=med_blink_med_buzz & color1=1) : 4*N1;
  (s1=med_blink_med_buzz & color1=2) : 5*N1;
  (s1=med_blink_med_buzz & color1=3) : 7*N1;
  (s1=high_Led & color1=1) : 6*N1;
  (s1=high_Led & color1=2) : 8*N1;
  (s1=high_Led & color1=3) : 12*N1;
  (s1=med_Led & color1=1) : 3*N1;
  (s1=med_Led & color1=2) : 4*N1;
  (s1=med_Led & color1=3) : 6*N1;
  (s1=low_Led & color1=1) : 2*N1;
  (s1=low_Led & color1=2) : 2*N1;
  (s1=low_Led & color1=3) : 3*N1;

///////////////////////////////////
  s2=high_pitch : 2*N2;
  s2=med_pitch : 1*N2;
  s2=low_pitch : 0*N2;
  (s2=high_blink_high_buzz & color2=1) : 8*N2;
  (s2=high_blink_high_buzz & color2=2) : 10*N2;
  (s2=high_blink_high_buzz & color2=3) : 14*N2;
  (s2=high_blink_med_buzz & color2=1) : 7*N2;
  (s2=high_blink_med_buzz & color2=2) : 9*N2;
  (s2=high_blink_med_buzz & color2=3) : 13*N2;
  (s2=med_blink_high_buzz & color2=1) : 5*N2;
  (s2=med_blink_high_buzz & color2=2) : 6*N2;
  (s2=med_blink_high_buzz & color2=3) : 8*N2;
  (s2=med_blink_med_buzz & color2=1) : 4*N2;
  (s2=med_blink_med_buzz & color2=2) : 5*N2;
  (s2=med_blink_med_buzz & color2=3) : 7*N2;
  (s2=high_Led & color1=1) : 6*N2;
  (s2=high_Led & color1=2) : 8*N2;
  (s2=high_Led & color1=3) : 12*N2;
  (s2=med_Led & color1=1) : 3*N2;
  (s2=med_Led & color1=2) : 4*N2;
  (s2=med_Led & color1=3) : 6*N2;
  (s2=low_Led & color1=1) : 2*N2;
  (s2=low_Led & color1=2) : 2*N2;
  (s2=low_Led & color1=3) : 3*N2;
endrewards
