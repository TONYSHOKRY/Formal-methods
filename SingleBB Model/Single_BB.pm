mdp //Markov decision process

// this Model represent a Markov decision process for only one BB

////////////////////cluster sizes
const int N1 = 1;
//////////////////////////////

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
const int idle = 19;


/////////////////////////
//labels
label "failed" = (s1=task_failed) ;
label "success" = (s1=finish);
label "both_active" = active1 ;
label "high_energy" = (s1=high_blink_high_buzz | s1=high_Led | s1=high_pitch) ;
label "noHWEnv" =
  (hw_state=0 & sound=0 & move=0 & env_state1=0 );
label "low_energy" = (s1=med_blink_med_buzz | s1=low_Led | s1=low_pitch);
label "LED_color" = (color1!=0) ;





////////////////////////////////////////////////
//patterns coefficient
const double theta;// this is a ratio that in rectangle it manage to runs more blocks that in aline pattern by this value when the same power supply spot(edged)

const double betai1;// beta initial 1 only for cluster 1 to prevent the probabilities distribution violation from high intensiy to medium intensity
const double betai2;// beta initial 2 only for cluster 1 to prevent the probabilities distribution violation from high intensiy to low intensity

// base success prob for high_LED in cluster 1 (per color)
const double q_w_high_c1=0.7; // white, cluster1
const double q_b_high_c1=0.8; // blue,  cluster1
const double q_r_high_c1=0.9; // red,   cluster1
const double q_high_pitch_c1;//=0.9; // highpitch, cluster1 (assumed)
//////////////////////////////////////////////////////


//motif 
module pattern
Rec_motif1:bool init false;//means it is line patter
[Rec_1]Rec_motif1=false & s1=idle & task_counter=0  ->(Rec_motif1'=true);
endmodule

//hardware
module Hardware
  hw_state : [0..2] init 0;

 // Can be updated via some sync or monitoring
  [update_hw_med] hw_state=0 & task_counter<=2 & (s1> listen_off_moving_on & s1< task_failed) -> (hw_state'=1);
  [update_hw_high] hw_state=0 & task_counter<=2 & (s1> listen_off_moving_on & s1< task_failed)-> (hw_state'=2);
  [reset_hw] hw_state>0& task_counter<=2  &  (s1> task_failed ) -> (hw_state'=0);

endmodule

// Environment sensing: 0 = idle, 1 = MIC, 2 = ACC, 3 = BOTH
module Environment
  sound :[0..1] init 0;
  move :[0..1] init 0;
  env_state1 : [0..3] init 0;


  [sound_on] sound=0 & task_counter<2 &s1!=start_task & (env_state1=1)-> (sound'=1);
  [move_on] move=0 & task_counter<2  &s1!=start_task -> (move'=1);
  [sound_off] sound=1 & task_counter<2  &s1!=start_task -> (sound'=0);
  [move_off] move=1& task_counter<2  &s1!=start_task-> (move'=0);
  [sense_mic1] s1=start_task & env_state1!=1  -> (env_state1'=1);
  [sense_acc1] s1=start_task & env_state1!=2 -> (env_state1'=2);
  [sense_both1]s1=start_task & env_state1!=3 -> (env_state1'=3);

  [reset_env1] env_state1!=0  & (s1=finish | s1=task_failed) -> (env_state1'=0);
endmodule




module task_Controller
  task_counter : [0..2] init 0;
  active1 : bool init true;

  [activate_1] (s1=idle) & !active1 & task_counter=1 -> (active1'=true);
  [deactivate_1] (s1=idle) & active1 -> (active1'=false);
  [Task_increment] task_counter<2 & 
         (!active1 | (active1  & (s1=finish | s1=task_failed)))->(task_counter'=task_counter+1);
endmodule

// BB1: Cluster 1 of Blinky Blocks
module BB1
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

 
 // Probabilistic transitions (BB1-specific) LINE pattern syncroninzed with other clusters
///////////////////////////////////
//high pitch
  [step] s1=high_pitch & hw_state=2 & Rec_motif1=false    -> (s1'=low_pitch); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_pitch & hw_state=1 & Rec_motif1=false    -> (s1'=med_pitch);// Priority 2: hardware MED -> reduce one level
  [step] s1=high_pitch & hw_state=0 & env_state1=2  & move=1& Rec_motif1=false    ->(1-q_high_pitch_c1)/2 : (s1'=med_pitch) + 1-((1-q_high_pitch_c1)/2) : (s1'=low_pitch);// Priority 3: environment sensing
  [step] s1=high_pitch & hw_state=0 & env_state1= 0 & move=1& Rec_motif1=false   ->(1-q_high_pitch_c1) : (s1'=task_failed)+ q_high_pitch_c1/2 : (s1'=med_pitch) + q_high_pitch_c1/2 : (s1'=low_pitch);// when move (no direct intention of fallback), chance of failure occurs
  [step] s1=high_pitch & hw_state=0 & env_state1=1  & sound=1 & Rec_motif1=false   ->1-((1-q_high_pitch_c1)/2) : (s1'=med_pitch) + (1-q_high_pitch_c1)/2 : (s1'=low_pitch);
  [step] s1=high_pitch& hw_state=0   & (sound=0 & move=0) & Rec_motif1=false   -> q_high_pitch_c1  : (s1'=finish) + (1-q_high_pitch_c1)/2 : (s1'=med_pitch) + (1-q_high_pitch_c1)/2 : (s1'=low_pitch); // default

//med pitch
  [step] s1=med_pitch & (hw_state=2 |hw_state=1) & Rec_motif1=false    -> (s1'=low_pitch); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [step] s1=med_pitch & hw_state=0 & (env_state1=2 |env_state1=1)  &( sound=1 |move=1)& Rec_motif1=false    -> (s1'=low_pitch);// Priority 3: environment sensing
  [step] s1=med_pitch & hw_state=0 & env_state1= 0 & move=1& Rec_motif1=false   -> 1-(q_high_pitch_c1*betai1) : (s1'=task_failed)+ q_high_pitch_c1*betai1 : (s1'=low_pitch);// when move (no direct intention of fallback), chance of failure occurs
  [step] s1=med_pitch& hw_state=0  & (sound=0 & move=0)& Rec_motif1=false   -> q_high_pitch_c1*betai1 : (s1'=finish) + (1-(q_high_pitch_c1*betai1)) : (s1'=low_pitch);// default

//low pitch
  [step] s1=low_pitch & (hw_state=2 |env_state1=2)& Rec_motif1=false    -> (1-(q_high_pitch_c1*betai2)) : (s1'=finish) + q_high_pitch_c1*betai2 : (s1'=task_failed); 
  [step] s1=low_pitch & (hw_state=1 |env_state1=1)& Rec_motif1=false    -> (1-((q_high_pitch_c1*betai2)/2)) : (s1'=finish) + (q_high_pitch_c1*betai2)/2 : (s1'=task_failed); 
  [step] s1=low_pitch & hw_state=0 & Rec_motif1=false   -> q_high_pitch_c1*betai2 : (s1'=finish) + (1-(q_high_pitch_c1*betai2)) : (s1'=task_failed);//even if movement it is already very low consuming and it is the lowest state so failure exist but not as when a higher consuming state

//sensor-environment transition
  [step] s1=listen_on_moving_on & Rec_motif1=false   -> 0.95 : (s1'=finish) + 0.05 : (s1'=task_failed);
  [step] s1=listen_on_moving_off & Rec_motif1=false   -> 0.95 : (s1'=finish) + 0.05 : (s1'=task_failed);
  [step] s1=listen_off_moving_on& Rec_motif1=false    -> 0.95 : (s1'=finish) + 0.05 : (s1'=task_failed);

//high-high simultaneous actuating white
  [step] s1=high_blink_high_buzz &color1=3 & hw_state=2 & Rec_motif1=false    -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_blink_high_buzz &color1=3 & hw_state=1 & Rec_motif1=false    -> 0.5 : (s1'=high_blink_med_buzz) +0.5: (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level
  [step] s1=high_blink_high_buzz &color1=3 & hw_state=0 & env_state1=2  & move=1& Rec_motif1=false   ->0.5 : (s1'=med_blink_high_buzz) + 0.5 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=high_blink_high_buzz &color1=3 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=false   ->1-(q_w_high_c1*q_high_pitch_c1) : (s1'=task_failed)+(q_w_high_c1*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + (q_w_high_c1*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s1=high_blink_high_buzz &color1=3 & hw_state=0 & env_state1=1  & sound=1& Rec_motif1=false   ->q_w_high_c1*q_high_pitch_c1 : (s1'=high_blink_med_buzz) + 1-(q_w_high_c1*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [step] s1=high_blink_high_buzz &color1=3 & hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=false    -> q_w_high_c1*q_high_pitch_c1: (s1'=finish) + (1-(q_w_high_c1*q_high_pitch_c1))/3 : (s1'=high_blink_med_buzz) + (1-(q_w_high_c1*q_high_pitch_c1))/3 : (s1'=med_blink_high_buzz) + (1-(q_w_high_c1*q_high_pitch_c1))/3 : (s1'=med_blink_med_buzz); //default(Empirical)
//high-high simultaneous actuating blue
  [step] s1=high_blink_high_buzz &color1=2 & hw_state=2 & Rec_motif1=false    -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_blink_high_buzz &color1=2 & hw_state=1 & Rec_motif1=false    -> 0.5 : (s1'=high_blink_med_buzz) +0.5: (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level
  [step] s1=high_blink_high_buzz &color1=2 & hw_state=0 & env_state1=2  & move=1& Rec_motif1=false   ->0.5 : (s1'=med_blink_high_buzz) + 0.5 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=high_blink_high_buzz &color1=2 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=false   ->1-(q_b_high_c1*q_high_pitch_c1) : (s1'=task_failed)+(q_b_high_c1*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + (q_b_high_c1*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s1=high_blink_high_buzz &color1=2 & hw_state=0 & env_state1=1  & sound=1& Rec_motif1=false   ->q_b_high_c1*q_high_pitch_c1 : (s1'=high_blink_med_buzz) + 1-(q_b_high_c1*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [step] s1=high_blink_high_buzz &color1=2 & hw_state=0 & env_state1=0  & (sound=0 & move=0) & Rec_motif1=false   -> q_b_high_c1*q_high_pitch_c1: (s1'=finish) + (1-(q_b_high_c1*q_high_pitch_c1))/3 : (s1'=high_blink_med_buzz) + (1-(q_b_high_c1*q_high_pitch_c1))/3 : (s1'=med_blink_high_buzz) + (1-(q_b_high_c1*q_high_pitch_c1))/3 : (s1'=med_blink_med_buzz); //default(Empirical)
//high-high simultaneous actuating RED
  [step] s1=high_blink_high_buzz &color1=1 & hw_state=2 & Rec_motif1=false    -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_blink_high_buzz &color1=1 & hw_state=1 & Rec_motif1=false    -> 0.5 : (s1'=high_blink_med_buzz) +0.5: (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level
  [step] s1=high_blink_high_buzz &color1=1 & hw_state=0 & env_state1=2  & move=1& Rec_motif1=false   ->0.5 : (s1'=med_blink_high_buzz) + 0.5 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=high_blink_high_buzz &color1=1 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=false   ->1-(q_r_high_c1*q_high_pitch_c1) : (s1'=task_failed)+(q_r_high_c1*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + (q_r_high_c1*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s1=high_blink_high_buzz &color1=1 & hw_state=0 & env_state1=1  & sound=1& Rec_motif1=false   ->q_r_high_c1*q_high_pitch_c1 : (s1'=high_blink_med_buzz) + 1-(q_r_high_c1*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [step] s1=high_blink_high_buzz &color1=1 & hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=false    -> q_r_high_c1*q_high_pitch_c1: (s1'=finish) + (1-(q_r_high_c1*q_high_pitch_c1))/3 : (s1'=high_blink_med_buzz) + (1-(q_r_high_c1*q_high_pitch_c1))/3 : (s1'=med_blink_high_buzz) + (1-(q_r_high_c1*q_high_pitch_c1))/3 : (s1'=med_blink_med_buzz); //default(Empirical)

//high-med simultaneous actuating white
  [step] s1=high_blink_med_buzz &color1=3 & hw_state=2& Rec_motif1=false     -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_blink_med_buzz &color1=3 & hw_state=1 & Rec_motif1=false    -> 0.5 : (s1'=med_blink_med_buzz) + 0.5 : (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level /equal probabilities cause it deoends on user preferences
  [step] s1=high_blink_med_buzz &color1=3& hw_state=0 & env_state1=2  & move=1& Rec_motif1=false   ->1-(q_w_high_c1*q_high_pitch_c1) : (s1'=med_blink_high_buzz) + q_w_high_c1*q_high_pitch_c1 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=high_blink_high_buzz &color1=3 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=false   ->1-(q_w_high_c1*q_high_pitch_c1) : (s1'=task_failed)+(q_w_high_c1*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + (q_w_high_c1*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s1=high_blink_med_buzz &color1=3& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=false   ->q_w_high_c1*q_high_pitch_c1  : (s1'=med_blink_med_buzz) + 1-(q_w_high_c1*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [step] s1=high_blink_med_buzz &color1=3& hw_state=0 & env_state1=0  & (sound=0 & move=0) & Rec_motif1=false   -> q_w_high_c1*q_high_pitch_c1 : (s1'=finish)  + (1-(q_w_high_c1*q_high_pitch_c1))/2 : (s1'=med_blink_high_buzz) + (1-(q_w_high_c1*q_high_pitch_c1))/2 : (s1'=med_blink_med_buzz); //default(empirical)
 //high-med simultaneous actuating blue
   [step] s1=high_blink_med_buzz &color1=2 & hw_state=2 & Rec_motif1=false    -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_blink_med_buzz &color1=2 & hw_state=1 & Rec_motif1=false    -> 0.5 : (s1'=med_blink_med_buzz) + 0.5 : (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level /equal probabilities cause it deoends on user preferences
  [step] s1=high_blink_med_buzz &color1=2& hw_state=0 & env_state1=2  & move=1& Rec_motif1=false   ->1-(q_b_high_c1*q_high_pitch_c1) : (s1'=med_blink_high_buzz) + q_b_high_c1*q_high_pitch_c1 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=high_blink_high_buzz &color1=2 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=false   ->1-(q_b_high_c1*q_high_pitch_c1) : (s1'=task_failed)+(q_b_high_c1*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + (q_b_high_c1*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s1=high_blink_med_buzz &color1=2& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=false   ->q_b_high_c1*q_high_pitch_c1  : (s1'=med_blink_med_buzz) + 1-(q_b_high_c1*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [step] s1=high_blink_med_buzz &color1=2& hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=false    -> q_b_high_c1*q_high_pitch_c1 : (s1'=finish)  + (1-(q_b_high_c1*q_high_pitch_c1))/2 : (s1'=med_blink_high_buzz) + (1-(q_b_high_c1*q_high_pitch_c1))/2 : (s1'=med_blink_med_buzz); //default(empirical)
 
//high-med simultaneous actuating Red
  [step] s1=high_blink_med_buzz &color1=1 & hw_state=2 & Rec_motif1=false    -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_blink_med_buzz &color1=1 & hw_state=1 & Rec_motif1=false    -> 0.5 : (s1'=med_blink_med_buzz) + 0.5 : (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level /equal probabilities cause it deoends on user preferences
  [step] s1=high_blink_med_buzz &color1=1& hw_state=0 & env_state1=2  & move=1& Rec_motif1=false   ->1-(q_r_high_c1*q_high_pitch_c1) : (s1'=med_blink_high_buzz) + q_r_high_c1*q_high_pitch_c1 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=high_blink_high_buzz &color1=1 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=false   ->1-(q_r_high_c1*q_high_pitch_c1) : (s1'=task_failed)+(q_r_high_c1*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + (q_r_high_c1*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s1=high_blink_med_buzz &color1=1& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=false   ->q_r_high_c1*q_high_pitch_c1  : (s1'=med_blink_med_buzz) + 1-(q_r_high_c1*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [step] s1=high_blink_med_buzz &color1=1& hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=false    -> q_r_high_c1*q_high_pitch_c1 : (s1'=finish)  + (1-(q_r_high_c1*q_high_pitch_c1))/2 : (s1'=med_blink_high_buzz) + (1-(q_r_high_c1*q_high_pitch_c1))/2 : (s1'=med_blink_med_buzz); //default(empirical)
 

 //med-high simultaneous actuating white
  [step] s1=med_blink_high_buzz &color1=3 & (hw_state=2 | hw_state=1) & Rec_motif1=false    -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=med_blink_high_buzz &color1=3 & hw_state=0 & (env_state1=2 | env_state1=1)   & (sound=1 | move=1)& Rec_motif1=false   -> (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=med_blink_high_buzz &color1=3 & hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=false   -> q_w_high_c1*q_high_pitch_c1*betai1 : (s1'=finish)  + 1-(q_w_high_c1*q_high_pitch_c1*betai1) : (s1'=med_blink_med_buzz); //default(empirical)
  [step] s1=med_blink_high_buzz &color1=3 & hw_state=0 & env_state1=0  & ( move=1)& Rec_motif1=false   -> 1-(q_w_high_c1*q_high_pitch_c1*betai1) : (s1'=task_failed)  + q_w_high_c1*q_high_pitch_c1*betai1 : (s1'=med_blink_med_buzz); //when movement un-intentionally
 //med-high simultaneous actuating blue
  [step] s1=med_blink_high_buzz &color1=2 & (hw_state=2 | hw_state=1)& Rec_motif1=false     -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=med_blink_high_buzz &color1=2 & hw_state=0 & (env_state1=2 | env_state1=1)   & (sound=1 | move=1)& Rec_motif1=false   -> (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=med_blink_high_buzz &color1=2 & hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=false   -> q_b_high_c1*q_high_pitch_c1*betai1 : (s1'=finish)  + 1-(q_b_high_c1*q_high_pitch_c1*betai1) : (s1'=med_blink_med_buzz); //default(empirical)
  [step] s1=med_blink_high_buzz &color1=2 & hw_state=0 & env_state1=0  & ( move=1)& Rec_motif1=false   -> 1-(q_b_high_c1*q_high_pitch_c1*betai1) : (s1'=task_failed)  + q_b_high_c1*q_high_pitch_c1*betai1 : (s1'=med_blink_med_buzz); //when movement un-intentionally
 //med-high simultaneous actuating Red
  [step] s1=med_blink_high_buzz &color1=1 & (hw_state=2 | hw_state=1) & Rec_motif1=false    -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=med_blink_high_buzz &color1=1 & hw_state=0 & (env_state1=2 | env_state1=1)   & (sound=1 | move=1)& Rec_motif1=false   -> (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=med_blink_high_buzz &color1=1 & hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=false   -> q_r_high_c1*q_high_pitch_c1*betai1 : (s1'=finish)  + 1-(q_r_high_c1*q_high_pitch_c1*betai1) : (s1'=med_blink_med_buzz); //default(empirical)
  [step] s1=med_blink_high_buzz &color1=1 & hw_state=0 & env_state1=0  & ( move=1) & Rec_motif1=false   -> 1-(q_r_high_c1*q_high_pitch_c1*betai1) : (s1'=task_failed)  + q_r_high_c1*q_high_pitch_c1*betai1 : (s1'=med_blink_med_buzz); //when movement un-intentionally



//med-med simultaneous actuating white
  [step] s1=med_blink_med_buzz &color1=3& (hw_state=2 |env_state1=2)& Rec_motif1=false    -> 1-(q_w_high_c1*q_high_pitch_c1*betai2) : (s1'=finish) + (q_w_high_c1*q_high_pitch_c1*betai2) : (s1'=task_failed); 
  [step] s1=med_blink_med_buzz &color1=3& (hw_state=1 |env_state1=1) & Rec_motif1=false   -> 0.5 : (s1'=finish) + 0.5 : (s1'=task_failed); 
  [step] s1=med_blink_med_buzz &color1=3& hw_state=0& Rec_motif1=false   -> (q_w_high_c1*q_high_pitch_c1*betai2) : (s1'=finish) + 1-(q_w_high_c1*q_high_pitch_c1*betai2) : (s1'=task_failed);//empirical data
//med-med simultaneous actuating Blue
[step] s1=med_blink_med_buzz &color1=2& (hw_state=2 |env_state1=2) & Rec_motif1=false   -> 1-(q_b_high_c1*q_high_pitch_c1*betai2) : (s1'=finish) + (q_b_high_c1*q_high_pitch_c1*betai2) : (s1'=task_failed); 
  [step] s1=med_blink_med_buzz &color1=2& (hw_state=1 |env_state1=1)& Rec_motif1=false    -> 0.5 : (s1'=finish) + 0.5 : (s1'=task_failed); 
  [step] s1=med_blink_med_buzz &color1=2& hw_state=0& Rec_motif1=false   -> (q_b_high_c1*q_high_pitch_c1*betai2) : (s1'=finish) + 1-(q_b_high_c1*q_high_pitch_c1*betai2) : (s1'=task_failed);//empirical data
//med-med simultaneous actuating Red
  [step] s1=med_blink_med_buzz &color1=1& (hw_state=2 |env_state1=2)& Rec_motif1=false    -> 1-(q_r_high_c1*q_high_pitch_c1*betai2) : (s1'=finish) + (q_r_high_c1*q_high_pitch_c1*betai2) : (s1'=task_failed); 
  [step] s1=med_blink_med_buzz &color1=1& (hw_state=1 |env_state1=1) & Rec_motif1=false   -> 0.5 : (s1'=finish) + 0.5 : (s1'=task_failed); 
  [step] s1=med_blink_med_buzz &color1=1& hw_state=0 & Rec_motif1=false   -> (q_r_high_c1*q_high_pitch_c1*betai2) : (s1'=finish) + 1-(q_r_high_c1*q_high_pitch_c1*betai2) : (s1'=task_failed);//empirical data

//High LED white
  [step] s1=high_Led  &color1=3& hw_state=2 & Rec_motif1=false    -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_Led  &color1=3& hw_state=1 & Rec_motif1=false    -> (s1'=med_Led);// Priority 2: hardware MED -> reduce one level
  [step] s1=high_Led  &color1=3& hw_state=0 & env_state1=2 & move=1& Rec_motif1=false   ->0.5  : (s1'=med_Led) + 0.5 : (s1'=low_Led);// Priority 3: environment sensing
  [step] s1=high_Led  &color1=3& hw_state=0 & env_state1=0 & move=1& Rec_motif1=false   ->(1-q_w_high_c1) : (s1'=task_failed)  +q_w_high_c1/2  : (s1'=med_Led) + q_w_high_c1/2 : (s1'=low_Led);// when movement un-intentionally
  [step] s1=high_Led  &color1=3& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=false   ->0.5 : (s1'=med_Led) + 0.5 : (s1'=low_Led);
  [step] s1=high_Led  &color1=3& hw_state=0 & env_state1=0 & (sound=0 & move=0) & Rec_motif1=false   -> q_w_high_c1 : (s1'=finish) + (1-q_w_high_c1)/2 : (s1'=med_Led) + (1-q_w_high_c1)/2 : (s1'=low_Led); // default (empirical)
//High LED Blue
  [step] s1=high_Led  &color1=2& hw_state=2 & Rec_motif1=false    -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_Led  &color1=2& hw_state=1 & Rec_motif1=false    -> (s1'=med_Led);// Priority 2: hardware MED -> reduce one level
  [step] s1=high_Led  &color1=2& hw_state=0 & env_state1=2 & move=1& Rec_motif1=false   ->0.5  : (s1'=med_Led) + 0.5 : (s1'=low_Led);// Priority 3: environment sensing
  [step] s1=high_Led  &color1=2& hw_state=0 & env_state1=0 & move=1& Rec_motif1=false   ->(1-q_b_high_c1) : (s1'=task_failed)  +q_b_high_c1/2  : (s1'=med_Led) + q_b_high_c1/2 : (s1'=low_Led);// when movement un-intentionally
  [step] s1=high_Led  &color1=2& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=false   ->0.5 : (s1'=med_Led) + 0.5 : (s1'=low_Led);
  [step] s1=high_Led  &color1=2& hw_state=0 & env_state1=0 & (sound=0 & move=0)& Rec_motif1=false    -> q_b_high_c1 : (s1'=finish) + (1-q_b_high_c1)/2 : (s1'=med_Led) + (1-q_b_high_c1)/2 : (s1'=low_Led); // default (empirical)
//High LED Red
   [step] s1=high_Led  &color1=1& hw_state=2 & Rec_motif1=false    -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_Led  &color1=1& hw_state=1 & Rec_motif1=false    -> (s1'=med_Led);// Priority 2: hardware MED -> reduce one level
  [step] s1=high_Led  &color1=1& hw_state=0 & env_state1=2 & move=1& Rec_motif1=false   ->0.5  : (s1'=med_Led) + 0.5 : (s1'=low_Led);// Priority 3: environment sensing
  [step] s1=high_Led  &color1=1& hw_state=0 & env_state1=0 & move=1& Rec_motif1=false   ->(1-q_r_high_c1) : (s1'=task_failed)  +q_r_high_c1/2  : (s1'=med_Led) + q_r_high_c1/2 : (s1'=low_Led);// when movement un-intentionally
  [step] s1=high_Led  &color1=1& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=false   ->0.5 : (s1'=med_Led) + 0.5 : (s1'=low_Led);
  [step] s1=high_Led  &color1=1& hw_state=0 & env_state1=0 & (sound=0 & move=0)& Rec_motif1=false    -> q_r_high_c1 : (s1'=finish) + (1-q_r_high_c1)/2 : (s1'=med_Led) + (1-q_r_high_c1)/2 : (s1'=low_Led); // default (empirical)

//Med LED white
  [step] s1=med_Led  &color1=3& (hw_state=2 |hw_state=1) & Rec_motif1=false    -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [step] s1=med_Led  &color1=3& hw_state=0 & (env_state1=2 |env_state1=1)  & (sound=1 | move=1)& Rec_motif1=false   -> (s1'=low_Led);// Priority 3: environment sensing
  [step] s1=med_Led  &color1=3& hw_state=0 & (sound=0 | move=0)& Rec_motif1=false   -> q_w_high_c1*betai1 : (s1'=finish) + 1-(q_w_high_c1*betai1) : (s1'=low_Led);// default (empirical)
  [step] s1=med_Led  &color1=3& hw_state=0 & (move=1)& Rec_motif1=false   -> 1-(q_w_high_c1*betai1) : (s1'=task_failed) + q_w_high_c1*betai1 : (s1'=low_Led);// when movement un-intentionally
//Med LED Blue
  [step] s1=med_Led   &color1=2& (hw_state=2 |hw_state=1) & Rec_motif1=false    -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [step] s1=med_Led  &color1=2& hw_state=0 & (env_state1=2 |env_state1=1)  & (sound=1 | move=1)& Rec_motif1=false   -> (s1'=low_Led);// Priority 3: environment sensing
  [step] s1=med_Led  &color1=2& hw_state=0 & (sound=0 | move=0)& Rec_motif1=false   -> q_b_high_c1*betai1 : (s1'=finish) + 1-(q_b_high_c1*betai1) : (s1'=low_Led);// default (empirical)
  [step] s1=med_Led  &color1=2& hw_state=0 & (move=1)& Rec_motif1=false   -> 1-(q_b_high_c1*betai1) : (s1'=task_failed) + q_b_high_c1*betai1 : (s1'=low_Led);// when movement un-intentionally
//Med LED Red
  [step] s1=med_Led   &color1=1& (hw_state=2 |hw_state=1) & Rec_motif1=false    -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [step] s1=med_Led  &color1=1& hw_state=0 & (env_state1=2 |env_state1=1)  & (sound=1 | move=1)& Rec_motif1=false   -> (s1'=low_Led);// Priority 3: environment sensing
  [step] s1=med_Led  &color1=1& hw_state=0 & (sound=0 | move=0)& Rec_motif1=false   -> q_r_high_c1*betai1 : (s1'=finish) + 1-(q_r_high_c1*betai1) : (s1'=low_Led);// default (empirical)
  [step] s1=med_Led  &color1=1& hw_state=0 & (move=1)& Rec_motif1=false   -> 1-(q_r_high_c1*betai1) : (s1'=task_failed) + q_r_high_c1*betai1 : (s1'=low_Led);// when movement un-intentionally

//low led white
  [step] s1=low_Led &color1=3& (hw_state=2 |env_state1=2) & Rec_motif1=false   -> 1-(q_w_high_c1*betai2) : (s1'=finish) + q_w_high_c1*betai2 : (s1'=task_failed); 
  [step] s1=low_Led &color1=3& (hw_state=0 |hw_state=1)& (env_state1=1 |env_state1=0)& Rec_motif1=false   -> q_w_high_c1*betai2 : (s1'=finish) + 1-(q_w_high_c1*betai2) : (s1'=task_failed);// default
//low led Blue
  [step] s1=low_Led &color1=2& (hw_state=2 |env_state1=2)& Rec_motif1=false    -> 1-(q_b_high_c1*betai2) : (s1'=finish) + q_b_high_c1*betai2 : (s1'=task_failed); 
  [step] s1=low_Led &color1=2& (hw_state=0 |hw_state=1)& (env_state1=1 |env_state1=0)& Rec_motif1=false   -> q_b_high_c1*betai2 : (s1'=finish) + 1-(q_b_high_c1*betai2) : (s1'=task_failed);// default
//low led Red
   [step] s1=low_Led &color1=1& (hw_state=2 |env_state1=2)& Rec_motif1=false    -> 1-(q_r_high_c1*betai2) : (s1'=finish) + q_r_high_c1*betai2 : (s1'=task_failed); 
  [step] s1=low_Led &color1=1& (hw_state=0 |hw_state=1)& (env_state1=1 |env_state1=0)& Rec_motif1=false   -> q_r_high_c1*betai2 : (s1'=finish) + 1-(q_r_high_c1*betai2) : (s1'=task_failed);// default
//////////////////////////////
  // Probabilistic transitions (BB1-specific)RECTANGLE pattern syncroninzed with other clusters
///////////////////////////////
 //high pitch
  [step] s1=high_pitch & hw_state=2 & Rec_motif1=true    -> (s1'=low_pitch); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_pitch & hw_state=1 & Rec_motif1=true    -> (s1'=med_pitch);// Priority 2: hardware MED -> reduce one level
  [step] s1=high_pitch & hw_state=0 & env_state1=2  & move=1& Rec_motif1=true    ->(1-q_high_pitch_c1)/2 : (s1'=med_pitch) + 1-((1-q_high_pitch_c1)/2) : (s1'=low_pitch);// Priority 3: environment sensing
  [step] s1=high_pitch & hw_state=0 & env_state1= 0 & move=1& Rec_motif1=true   ->(1-q_high_pitch_c1) : (s1'=task_failed)+ q_high_pitch_c1/2 : (s1'=med_pitch) + q_high_pitch_c1/2 : (s1'=low_pitch);// when move (no direct intention of fallback), chance of failure occurs
  [step] s1=high_pitch & hw_state=0 & env_state1=1  & sound=1 & Rec_motif1=true   ->1-((1-q_high_pitch_c1)/2) : (s1'=med_pitch) + (1-q_high_pitch_c1)/2 : (s1'=low_pitch);
  [step] s1=high_pitch& hw_state=0   & (sound=0 & move=0) & Rec_motif1=true   -> q_high_pitch_c1  : (s1'=finish) + (1-q_high_pitch_c1)/2 : (s1'=med_pitch) + (1-q_high_pitch_c1)/2 : (s1'=low_pitch); // default

//med pitch
  [step] s1=med_pitch & (hw_state=2 |hw_state=1) & Rec_motif1=true    -> (s1'=low_pitch); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [step] s1=med_pitch & hw_state=0 & (env_state1=2 |env_state1=1)  &( sound=1 |move=1)& Rec_motif1=true    -> (s1'=low_pitch);// Priority 3: environment sensing
  [step] s1=med_pitch & hw_state=0 & env_state1= 0 & move=1& Rec_motif1=true   -> 1-(q_high_pitch_c1*betai1) : (s1'=task_failed)+ q_high_pitch_c1*betai1 : (s1'=low_pitch);// when move (no direct intention of fallback), chance of failure occurs
  [step] s1=med_pitch& hw_state=0  & (sound=0 & move=0)& Rec_motif1=true   -> q_high_pitch_c1*betai1 : (s1'=finish) + (1-(q_high_pitch_c1*betai1)) : (s1'=low_pitch);// default

//low pitch
  [step] s1=low_pitch & (hw_state=2 |env_state1=2)& Rec_motif1=true    -> (1-(q_high_pitch_c1*betai2)) : (s1'=finish) + q_high_pitch_c1*betai2 : (s1'=task_failed); 
  [step] s1=low_pitch & (hw_state=1 |env_state1=1)& Rec_motif1=true   -> (1-((q_high_pitch_c1*betai2)/2)) : (s1'=finish) + (q_high_pitch_c1*betai2)/2 : (s1'=task_failed); 
  [step] s1=low_pitch & hw_state=0 & Rec_motif1=true   -> q_high_pitch_c1*betai2 : (s1'=finish) + (1-(q_high_pitch_c1*betai2)) : (s1'=task_failed);//even if movement it is already very low consuming and it is the lowest state so failure exist but not as when a higher consuming state

//sensor-environment transition
  [step] s1=listen_on_moving_on & Rec_motif1=true   -> 0.95 : (s1'=finish) + 0.05 : (s1'=task_failed);
  [step] s1=listen_on_moving_off & Rec_motif1=true   -> 0.95 : (s1'=finish) + 0.05 : (s1'=task_failed);
  [step] s1=listen_off_moving_on& Rec_motif1=true    -> 0.95 : (s1'=finish) + 0.05 : (s1'=task_failed);

//high-high simultaneous actuating white
  [step] s1=high_blink_high_buzz &color1=3 & hw_state=2 & Rec_motif1=true    -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_blink_high_buzz &color1=3 & hw_state=1 & Rec_motif1=true    -> 0.5 : (s1'=high_blink_med_buzz) +0.5: (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level
  [step] s1=high_blink_high_buzz &color1=3 & hw_state=0 & env_state1=2  & move=1& Rec_motif1=true   ->0.5 : (s1'=med_blink_high_buzz) + 0.5 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=high_blink_high_buzz &color1=3 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=true   ->1-((q_w_high_c1*theta)*q_high_pitch_c1) : (s1'=task_failed)+((q_w_high_c1*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + ((q_w_high_c1*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s1=high_blink_high_buzz &color1=3 & hw_state=0 & env_state1=1  & sound=1& Rec_motif1=true   ->(q_w_high_c1*theta)*q_high_pitch_c1 : (s1'=high_blink_med_buzz) + 1-((q_w_high_c1*theta)*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [step] s1=high_blink_high_buzz &color1=3 & hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=true    -> (q_w_high_c1*theta)*q_high_pitch_c1: (s1'=finish) + (1-((q_w_high_c1*theta)*q_high_pitch_c1))/3 : (s1'=high_blink_med_buzz) + (1-((q_w_high_c1*theta)*q_high_pitch_c1))/3 : (s1'=med_blink_high_buzz) + (1-((q_w_high_c1*theta)*q_high_pitch_c1))/3 : (s1'=med_blink_med_buzz); //default(Empirical)
//high-high simultaneous actuating blue
  [step] s1=high_blink_high_buzz &color1=2 & hw_state=2 & Rec_motif1=true    -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_blink_high_buzz &color1=2 & hw_state=1 & Rec_motif1=true    -> 0.5 : (s1'=high_blink_med_buzz) +0.5: (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level
  [step] s1=high_blink_high_buzz &color1=2 & hw_state=0 & env_state1=2  & move=1& Rec_motif1=true   ->0.5 : (s1'=med_blink_high_buzz) + 0.5 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=high_blink_high_buzz &color1=2 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=true   ->1-((q_b_high_c1*theta)*q_high_pitch_c1) : (s1'=task_failed)+((q_b_high_c1*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + ((q_b_high_c1*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s1=high_blink_high_buzz &color1=2 & hw_state=0 & env_state1=1  & sound=1& Rec_motif1=true   ->(q_b_high_c1*theta)*q_high_pitch_c1 : (s1'=high_blink_med_buzz) + 1-((q_b_high_c1*theta)*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [step] s1=high_blink_high_buzz &color1=2 & hw_state=0 & env_state1=0  & (sound=0 & move=0) & Rec_motif1=true   -> (q_b_high_c1*theta)*q_high_pitch_c1: (s1'=finish) + (1-((q_b_high_c1*theta)*q_high_pitch_c1))/3 : (s1'=high_blink_med_buzz) + (1-((q_b_high_c1*theta)*q_high_pitch_c1))/3 : (s1'=med_blink_high_buzz) + (1-((q_b_high_c1*theta)*q_high_pitch_c1))/3 : (s1'=med_blink_med_buzz); //default(Empirical)
//high-high simultaneous actuating RED
  [step] s1=high_blink_high_buzz &color1=1 & hw_state=2 & Rec_motif1=true    -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_blink_high_buzz &color1=1 & hw_state=1 & Rec_motif1=true    -> 0.5 : (s1'=high_blink_med_buzz) +0.5: (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level
  [step] s1=high_blink_high_buzz &color1=1 & hw_state=0 & env_state1=2  & move=1& Rec_motif1=true   ->0.5 : (s1'=med_blink_high_buzz) + 0.5 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=high_blink_high_buzz &color1=1 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=true   ->1-((q_r_high_c1*theta)*q_high_pitch_c1) : (s1'=task_failed)+((q_r_high_c1*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + ((q_r_high_c1*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s1=high_blink_high_buzz &color1=1 & hw_state=0 & env_state1=1  & sound=1& Rec_motif1=true   ->(q_r_high_c1*theta)*q_high_pitch_c1 : (s1'=high_blink_med_buzz) + 1-((q_r_high_c1*theta)*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [step] s1=high_blink_high_buzz &color1=1 & hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=true    -> (q_r_high_c1*theta)*q_high_pitch_c1: (s1'=finish) + (1-((q_r_high_c1*theta)*q_high_pitch_c1))/3 : (s1'=high_blink_med_buzz) + (1-((q_r_high_c1*theta)*q_high_pitch_c1))/3 : (s1'=med_blink_high_buzz) + (1-((q_r_high_c1*theta)*q_high_pitch_c1))/3 : (s1'=med_blink_med_buzz); //default(Empirical)

//high-med simultaneous actuating white
  [step] s1=high_blink_med_buzz &color1=3 & hw_state=2& Rec_motif1=true     -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_blink_med_buzz &color1=3 & hw_state=1 & Rec_motif1=true    -> 0.5 : (s1'=med_blink_med_buzz) + 0.5 : (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level /equal probabilities cause it deoends on user preferences
  [step] s1=high_blink_med_buzz &color1=3& hw_state=0 & env_state1=2  & move=1& Rec_motif1=true   ->1-((q_w_high_c1*theta)*q_high_pitch_c1) : (s1'=med_blink_high_buzz) + (q_w_high_c1*theta)*q_high_pitch_c1 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=high_blink_high_buzz &color1=3 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=true   ->1-((q_w_high_c1*theta)*q_high_pitch_c1) : (s1'=task_failed)+((q_w_high_c1*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + ((q_w_high_c1*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s1=high_blink_med_buzz &color1=3& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=true   ->(q_w_high_c1*theta)*q_high_pitch_c1  : (s1'=med_blink_med_buzz) + 1-((q_w_high_c1*theta)*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [step] s1=high_blink_med_buzz &color1=3& hw_state=0 & env_state1=0  & (sound=0 & move=0) & Rec_motif1=true   -> (q_w_high_c1*theta)*q_high_pitch_c1 : (s1'=finish)  + (1-((q_w_high_c1*theta)*q_high_pitch_c1))/2 : (s1'=med_blink_high_buzz) + (1-((q_w_high_c1*theta)*q_high_pitch_c1))/2 : (s1'=med_blink_med_buzz); //default(empirical)
 //high-med simultaneous actuating blue
   [step] s1=high_blink_med_buzz &color1=2 & hw_state=2 & Rec_motif1=true    -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_blink_med_buzz &color1=2 & hw_state=1 & Rec_motif1=true    -> 0.5 : (s1'=med_blink_med_buzz) + 0.5 : (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level /equal probabilities cause it deoends on user preferences
  [step] s1=high_blink_med_buzz &color1=2& hw_state=0 & env_state1=2  & move=1& Rec_motif1=true   ->1-((q_b_high_c1*theta)*q_high_pitch_c1) : (s1'=med_blink_high_buzz) + (q_b_high_c1*theta)*q_high_pitch_c1 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=high_blink_high_buzz &color1=2 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=true   ->1-((q_b_high_c1*theta)*q_high_pitch_c1) : (s1'=task_failed)+((q_b_high_c1*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + ((q_b_high_c1*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s1=high_blink_med_buzz &color1=2& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=true   ->(q_b_high_c1*theta)*q_high_pitch_c1  : (s1'=med_blink_med_buzz) + 1-((q_b_high_c1*theta)*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [step] s1=high_blink_med_buzz &color1=2& hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=true    -> (q_b_high_c1*theta)*q_high_pitch_c1 : (s1'=finish)  + (1-((q_b_high_c1*theta)*q_high_pitch_c1))/2 : (s1'=med_blink_high_buzz) + (1-((q_b_high_c1*theta)*q_high_pitch_c1))/2 : (s1'=med_blink_med_buzz); //default(empirical)
 
//high-med simultaneous actuating Red
  [step] s1=high_blink_med_buzz &color1=1 & hw_state=2 & Rec_motif1=true    -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_blink_med_buzz &color1=1 & hw_state=1 & Rec_motif1=true    -> 0.5 : (s1'=med_blink_med_buzz) + 0.5 : (s1'=med_blink_high_buzz);// Priority 2: hardware MED -> reduce one level /equal probabilities cause it deoends on user preferences
  [step] s1=high_blink_med_buzz &color1=1& hw_state=0 & env_state1=2  & move=1& Rec_motif1=true   ->1-((q_r_high_c1*theta)*q_high_pitch_c1) : (s1'=med_blink_high_buzz) + (q_r_high_c1*theta)*q_high_pitch_c1 : (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=high_blink_high_buzz &color1=1 & hw_state=0 & env_state1=0  & move=1& Rec_motif1=true   ->1-((q_r_high_c1*theta)*q_high_pitch_c1) : (s1'=task_failed)+((q_r_high_c1*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_high_buzz) + ((q_r_high_c1*theta)*q_high_pitch_c1)/2 : (s1'=med_blink_med_buzz);// when move (no direct intention of fallback), chance of failure occurs
  [step] s1=high_blink_med_buzz &color1=1& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=true   ->(q_r_high_c1*theta)*q_high_pitch_c1  : (s1'=med_blink_med_buzz) + 1-((q_r_high_c1*theta)*q_high_pitch_c1) : (s1'=med_blink_high_buzz);
  [step] s1=high_blink_med_buzz &color1=1& hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=true    -> (q_r_high_c1*theta)*q_high_pitch_c1 : (s1'=finish)  + (1-((q_r_high_c1*theta)*q_high_pitch_c1))/2 : (s1'=med_blink_high_buzz) + (1-((q_r_high_c1*theta)*q_high_pitch_c1))/2 : (s1'=med_blink_med_buzz); //default(empirical)
 

 //med-high simultaneous actuating white
  [step] s1=med_blink_high_buzz &color1=3 & (hw_state=2 | hw_state=1) & Rec_motif1=true    -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=med_blink_high_buzz &color1=3 & hw_state=0 & (env_state1=2 | env_state1=1)   & (sound=1 | move=1)& Rec_motif1=true   -> (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=med_blink_high_buzz &color1=3 & hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=true   -> (q_w_high_c1*theta)*q_high_pitch_c1*betai1 : (s1'=finish)  + 1-((q_w_high_c1*theta)*q_high_pitch_c1*betai1) : (s1'=med_blink_med_buzz); //default(empirical)
  [step] s1=med_blink_high_buzz &color1=3 & hw_state=0 & env_state1=0  & ( move=1)& Rec_motif1=true   -> 1-((q_w_high_c1*theta)*q_high_pitch_c1*betai1) : (s1'=task_failed)  + (q_w_high_c1*theta)*q_high_pitch_c1*betai1 : (s1'=med_blink_med_buzz); //when movement un-intentionally
 //med-high simultaneous actuating blue
  [step] s1=med_blink_high_buzz &color1=2 & (hw_state=2 | hw_state=1)& Rec_motif1=true     -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=med_blink_high_buzz &color1=2 & hw_state=0 & (env_state1=2 | env_state1=1)   & (sound=1 | move=1)& Rec_motif1=true   -> (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=med_blink_high_buzz &color1=2 & hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=true   -> (q_b_high_c1*theta)*q_high_pitch_c1*betai1 : (s1'=finish)  + 1-((q_b_high_c1*theta)*q_high_pitch_c1*betai1) : (s1'=med_blink_med_buzz); //default(empirical)
  [step] s1=med_blink_high_buzz &color1=2 & hw_state=0 & env_state1=0  & ( move=1)& Rec_motif1=true   -> 1-((q_b_high_c1*theta)*q_high_pitch_c1*betai1) : (s1'=task_failed)  + (q_b_high_c1*theta)*q_high_pitch_c1*betai1 : (s1'=med_blink_med_buzz); //when movement un-intentionally
 //med-high simultaneous actuating Red
  [step] s1=med_blink_high_buzz &color1=1 & (hw_state=2 | hw_state=1) & Rec_motif1=true    -> (s1'=med_blink_med_buzz); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=med_blink_high_buzz &color1=1 & hw_state=0 & (env_state1=2 | env_state1=1)   & (sound=1 | move=1)& Rec_motif1=true   -> (s1'=med_blink_med_buzz);// Priority 3: environment sensing
  [step] s1=med_blink_high_buzz &color1=1 & hw_state=0 & env_state1=0  & (sound=0 & move=0)& Rec_motif1=true   -> (q_r_high_c1*theta)*q_high_pitch_c1*betai1 : (s1'=finish)  + 1-((q_r_high_c1*theta)*q_high_pitch_c1*betai1) : (s1'=med_blink_med_buzz); //default(empirical)
  [step] s1=med_blink_high_buzz &color1=1 & hw_state=0 & env_state1=0  & ( move=1) & Rec_motif1=true   -> 1-((q_r_high_c1*theta)*q_high_pitch_c1*betai1) : (s1'=task_failed)  + (q_r_high_c1*theta)*q_high_pitch_c1*betai1 : (s1'=med_blink_med_buzz); //when movement un-intentionally



//med-med simultaneous actuating white
  [step] s1=med_blink_med_buzz &color1=3& (hw_state=2 |env_state1=2)& Rec_motif1=true    -> 1-((q_w_high_c1*theta)*q_high_pitch_c1*betai2) : (s1'=finish) + ((q_w_high_c1*theta)*q_high_pitch_c1*betai2) : (s1'=task_failed); 
  [step] s1=med_blink_med_buzz &color1=3& (hw_state=1 |env_state1=1) & Rec_motif1=true   -> 0.5 : (s1'=finish) + 0.5 : (s1'=task_failed); 
  [step] s1=med_blink_med_buzz &color1=3& hw_state=0& Rec_motif1=true   -> ((q_w_high_c1*theta)*q_high_pitch_c1*betai2) : (s1'=finish) + 1-((q_w_high_c1*theta)*q_high_pitch_c1*betai2) : (s1'=task_failed);//empirical data
//med-med simultaneous actuating Blue
[step] s1=med_blink_med_buzz &color1=2& (hw_state=2 |env_state1=2) & Rec_motif1=true   -> 1-((q_b_high_c1*theta)*q_high_pitch_c1*betai2) : (s1'=finish) + ((q_b_high_c1*theta)*q_high_pitch_c1*betai2) : (s1'=task_failed); 
  [step] s1=med_blink_med_buzz &color1=2& (hw_state=1 |env_state1=1)& Rec_motif1=true    -> 0.5 : (s1'=finish) + 0.5 : (s1'=task_failed); 
  [step] s1=med_blink_med_buzz &color1=2& hw_state=0& Rec_motif1=true   -> ((q_b_high_c1*theta)*q_high_pitch_c1*betai2) : (s1'=finish) + 1-((q_b_high_c1*theta)*q_high_pitch_c1*betai2) : (s1'=task_failed);//empirical data
//med-med simultaneous actuating Red
  [step] s1=med_blink_med_buzz &color1=1& (hw_state=2 |env_state1=2)& Rec_motif1=true    -> 1-((q_r_high_c1*theta)*q_high_pitch_c1*betai2) : (s1'=finish) + ((q_r_high_c1*theta)*q_high_pitch_c1*betai2) : (s1'=task_failed); 
  [step] s1=med_blink_med_buzz &color1=1& (hw_state=1 |env_state1=1) & Rec_motif1=true   -> 0.5 : (s1'=finish) + 0.5 : (s1'=task_failed); 
  [step] s1=med_blink_med_buzz &color1=1& hw_state=0 & Rec_motif1=true   -> ((q_r_high_c1*theta)*q_high_pitch_c1*betai2) : (s1'=finish) + 1-((q_r_high_c1*theta)*q_high_pitch_c1*betai2) : (s1'=task_failed);//empirical data

//High LED white
  [step] s1=high_Led  &color1=3& hw_state=2 & Rec_motif1=true    -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_Led  &color1=3& hw_state=1 & Rec_motif1=true    -> (s1'=med_Led);// Priority 2: hardware MED -> reduce one level
  [step] s1=high_Led  &color1=3& hw_state=0 & env_state1=2 & move=1& Rec_motif1=true   ->0.5  : (s1'=med_Led) + 0.5 : (s1'=low_Led);// Priority 3: environment sensing
  [step] s1=high_Led  &color1=3& hw_state=0 & env_state1=0 & move=1& Rec_motif1=true   ->(1-(q_w_high_c1*theta)) : (s1'=task_failed)  +(q_w_high_c1*theta)/2  : (s1'=med_Led) + (q_w_high_c1*theta)/2 : (s1'=low_Led);// when movement un-intentionally
  [step] s1=high_Led  &color1=3& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=true   ->0.5 : (s1'=med_Led) + 0.5 : (s1'=low_Led);
  [step] s1=high_Led  &color1=3& hw_state=0 & env_state1=0 & (sound=0 & move=0) & Rec_motif1=true   -> (q_w_high_c1*theta) : (s1'=finish) + (1-(q_w_high_c1*theta))/2 : (s1'=med_Led) + (1-(q_w_high_c1*theta))/2 : (s1'=low_Led); // default (empirical)
//High LED Blue
  [step] s1=high_Led  &color1=2& hw_state=2 & Rec_motif1=true    -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_Led  &color1=2& hw_state=1 & Rec_motif1=true    -> (s1'=med_Led);// Priority 2: hardware MED -> reduce one level
  [step] s1=high_Led  &color1=2& hw_state=0 & env_state1=2 & move=1& Rec_motif1=true   ->0.5  : (s1'=med_Led) + 0.5 : (s1'=low_Led);// Priority 3: environment sensing
  [step] s1=high_Led  &color1=2& hw_state=0 & env_state1=0 & move=1& Rec_motif1=true   ->(1-(q_b_high_c1*theta)) : (s1'=task_failed)  +(q_b_high_c1*theta)/2  : (s1'=med_Led) + (q_b_high_c1*theta)/2 : (s1'=low_Led);// when movement un-intentionally
  [step] s1=high_Led  &color1=2& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=true   ->0.5 : (s1'=med_Led) + 0.5 : (s1'=low_Led);
  [step] s1=high_Led  &color1=2& hw_state=0 & env_state1=0 & (sound=0 & move=0)& Rec_motif1=true    -> (q_b_high_c1*theta) : (s1'=finish) + (1-(q_b_high_c1*theta))/2 : (s1'=med_Led) + (1-(q_b_high_c1*theta))/2 : (s1'=low_Led); // default (empirical)
//High LED Red
   [step] s1=high_Led  &color1=1& hw_state=2 & Rec_motif1=true    -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade
  [step] s1=high_Led  &color1=1& hw_state=1 & Rec_motif1=true    -> (s1'=med_Led);// Priority 2: hardware MED -> reduce one level
  [step] s1=high_Led  &color1=1& hw_state=0 & env_state1=2 & move=1& Rec_motif1=true   ->0.5  : (s1'=med_Led) + 0.5 : (s1'=low_Led);// Priority 3: environment sensing
  [step] s1=high_Led  &color1=1& hw_state=0 & env_state1=0 & move=1& Rec_motif1=true   ->(1-(q_r_high_c1*theta)) : (s1'=task_failed)  +(q_r_high_c1*theta)/2  : (s1'=med_Led) + (q_r_high_c1*theta)/2 : (s1'=low_Led);// when movement un-intentionally
  [step] s1=high_Led  &color1=1& hw_state=0 & env_state1=1  & sound=1& Rec_motif1=true   ->0.5 : (s1'=med_Led) + 0.5 : (s1'=low_Led);
  [step] s1=high_Led  &color1=1& hw_state=0 & env_state1=0 & (sound=0 & move=0)& Rec_motif1=true    -> (q_r_high_c1*theta) : (s1'=finish) + (1-(q_r_high_c1*theta))/2 : (s1'=med_Led) + (1-(q_r_high_c1*theta))/2 : (s1'=low_Led); // default (empirical)

//Med LED white
  [step] s1=med_Led  &color1=3& (hw_state=2 |hw_state=1) & Rec_motif1=true    -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [step] s1=med_Led  &color1=3& hw_state=0 & (env_state1=2 |env_state1=1)  & (sound=1 | move=1)& Rec_motif1=true   -> (s1'=low_Led);// Priority 3: environment sensing
  [step] s1=med_Led  &color1=3& hw_state=0 & (sound=0 | move=0)& Rec_motif1=true   -> (q_w_high_c1*theta)*betai1 : (s1'=finish) + 1-((q_w_high_c1*theta)*betai1) : (s1'=low_Led);// default (empirical)
  [step] s1=med_Led  &color1=3& hw_state=0 & (move=1)& Rec_motif1=true   -> 1-((q_w_high_c1*theta)*betai1) : (s1'=task_failed) + (q_w_high_c1*theta)*betai1 : (s1'=low_Led);// when movement un-intentionally
//Med LED Blue
  [step] s1=med_Led   &color1=2& (hw_state=2 |hw_state=1) & Rec_motif1=true    -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [step] s1=med_Led  &color1=2& hw_state=0 & (env_state1=2 |env_state1=1)  & (sound=1 | move=1)& Rec_motif1=true   -> (s1'=low_Led);// Priority 3: environment sensing
  [step] s1=med_Led  &color1=2& hw_state=0 & (sound=0 | move=0)& Rec_motif1=true   -> (q_b_high_c1*theta)*betai1 : (s1'=finish) + 1-((q_b_high_c1*theta)*betai1) : (s1'=low_Led);// default (empirical)
  [step] s1=med_Led  &color1=2& hw_state=0 & (move=1)& Rec_motif1=true   -> 1-((q_b_high_c1*theta)*betai1) : (s1'=task_failed) + (q_b_high_c1*theta)*betai1 : (s1'=low_Led);// when movement un-intentionally
//Med LED Red
  [step] s1=med_Led   &color1=1& (hw_state=2 |hw_state=1) & Rec_motif1=true    -> (s1'=low_Led); // Priority 1: hardware HIGH -> immediate downgrade // Priority 2: hardware MED -> reduce one level
  [step] s1=med_Led  &color1=1& hw_state=0 & (env_state1=2 |env_state1=1) &  (sound=1 | move=1)& Rec_motif1=true   -> (s1'=low_Led);// Priority 3: environment sensing
  [step] s1=med_Led  &color1=1& hw_state=0 & (sound=0 | move=0)& Rec_motif1=true   -> (q_r_high_c1*theta)*betai1 : (s1'=finish) + 1-((q_r_high_c1*theta)*betai1) : (s1'=low_Led);// default (empirical)
  [step] s1=med_Led  &color1=1& hw_state=0 & (move=1)& Rec_motif1=true   -> 1-((q_r_high_c1*theta)*betai1) : (s1'=task_failed) + (q_r_high_c1*theta)*betai1 : (s1'=low_Led);// when movement un-intentionally

//low led white
  [step] s1=low_Led &color1=3& (hw_state=2 |env_state1=2) & Rec_motif1=true   -> 1-((q_w_high_c1*theta)*betai2) : (s1'=finish) + (q_w_high_c1*theta)*betai2 : (s1'=task_failed); 
  [step] s1=low_Led &color1=3& (hw_state=0 |hw_state=1)& (env_state1=1 |env_state1=0)& Rec_motif1=true  -> (q_w_high_c1*theta)*betai2 : (s1'=finish) + 1-((q_w_high_c1*theta)*betai2) : (s1'=task_failed);// default
//low led Blue
  [step] s1=low_Led &color1=2& (hw_state=2 |env_state1=2)& Rec_motif1=true    -> 1-((q_b_high_c1*theta)*betai2) : (s1'=finish) + (q_b_high_c1*theta)*betai2 : (s1'=task_failed); 
  [step] s1=low_Led &color1=2& (hw_state=0 |hw_state=1)& (env_state1=1 |env_state1=0)& Rec_motif1=true   -> (q_b_high_c1*theta)*betai2 : (s1'=finish) + 1-((q_b_high_c1*theta)*betai2) : (s1'=task_failed);// default
//low led Red
   [step] s1=low_Led &color1=1& (hw_state=2 |env_state1=2)& Rec_motif1=true    -> 1-((q_r_high_c1*theta)*betai2) : (s1'=finish) + (q_r_high_c1*theta)*betai2 : (s1'=task_failed); 
  [step] s1=low_Led &color1=1& (hw_state=0 |hw_state=1)& (env_state1=1 |env_state1=0)& Rec_motif1=true   -> (q_r_high_c1*theta)*betai2 : (s1'=finish) + 1-((q_r_high_c1*theta)*betai2) : (s1'=task_failed);// default
//////////////////////////////

  // Reset on tick
  [Task_increment] s1=finish  -> (s1'=start_task) ;
  [Task_increment] s1=task_failed  -> (s1'=start_task) ;

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
endrewards
