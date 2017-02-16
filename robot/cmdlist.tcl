# cmdlist.tcl - command list for mkcmds.tcl

# InMotion2 robot system software for RTLinux

# Copyright 2003-2005 Interactive Motion Technologies, Inc.
# Cambridge, MA, USA
# http://www.interactive-motion.com
# All rights reserved

# the command list needs to be initiated with the locations of the shared
# memory struct members, after they are alloc'd.  You can't do this variable
# init the C declaration initializer, so here's a tcl script that pumps out
# the C code to do it at runtime.  This generates a .c source file that is,
# in turn, compiled.

# do not put any comments inside the cmdlist.

set cmdlist {
    {so_f64 shoulder_angle_cal &rob->shoulder.angle.cal 1}
    {so_u32 shoulder_angle_channel &rob->shoulder.angle.channel 1}
    {so_f64 shoulder_angle_degrees &rob->shoulder.angle.deg 1}
    {so_f64 shoulder_angle_rad &rob->shoulder.angle.rad 1}
    {so_f64 shoulder_angle_offset &rob->shoulder.angle.offset 1}
    {so_f64 shoulder_angle_xform &rob->shoulder.angle.xform 1}
    {so_u32 shoulder_torque_channel &rob->shoulder.torque.channel 1}
    {so_f64 shoulder_torque_offset &rob->shoulder.torque.offset 1}
    {so_f64 shoulder_torque_xform &rob->shoulder.torque.xform 1}
    {so_f64 shoulder_vel_cal &rob->shoulder.vel.cal 1}
    {so_u32 shoulder_vel_channel &rob->shoulder.vel.channel 1}
    {so_f64 shoulder_vel_offset &rob->shoulder.vel.offset 1}
    {so_f64 shoulder_vel_xform &rob->shoulder.vel.xform 1}
    {so_f64 shoulder_vel_rad &rob->shoulder.vel.rad 1}

    {so_f64 elbow_angle_cal &rob->elbow.angle.cal 1}
    {so_u32 elbow_angle_channel &rob->elbow.angle.channel 1}
    {so_f64 elbow_angle_degrees &rob->elbow.angle.deg 1}
    {so_f64 elbow_angle_rad &rob->elbow.angle.rad 1}
    {so_f64 elbow_angle_offset &rob->elbow.angle.offset 1}
    {so_f64 elbow_angle_xform &rob->elbow.angle.xform 1}
    {so_u32 elbow_torque_channel &rob->elbow.torque.channel 1}
    {so_f64 elbow_torque_offset &rob->elbow.torque.offset 1}
    {so_f64 elbow_torque_xform &rob->elbow.torque.xform 1}
    {so_f64 elbow_vel_cal &rob->elbow.vel.cal 1}
    {so_u32 elbow_vel_channel &rob->elbow.vel.channel 1}
    {so_f64 elbow_vel_offset &rob->elbow.vel.offset 1}
    {so_f64 elbow_vel_xform &rob->elbow.vel.xform 1}
    {so_f64 elbow_vel_rad &rob->elbow.vel.rad 1}

    {so_f64 wrist_diff_stiff &ob->wrist.diff_stiff 1}
    {so_f64 wrist_diff_damp &ob->wrist.diff_damp 1}
    {so_f64 wrist_diff_side_stiff &ob->wrist.diff_side_stiff 1}
    {so_f64 wrist_ps_stiff &ob->wrist.ps_stiff 1}
    {so_f64 wrist_ps_damp &ob->wrist.ps_damp 1}
    {so_f64 wrist_diff_gcomp &ob->wrist.diff_gcomp 1}
    {so_f64 wrist_ps_gcomp &ob->wrist.ps_gcomp 1}
    {so_f64 wrist_rl_pfomax &ob->wrist.rl_pfomax 1}
    {so_f64 wrist_rl_pfotest &ob->wrist.rl_pfotest 1}
    {so_u32 wrist_nocenter3d &ob->wrist.nocenter3d 1}
    {so_u32 wrist_ps_adap_going_up &ob->wrist.ps_adap_going_up 1}
    {so_f64 wrist_velmag &ob->wrist.velmag 1}
    {so_u32 wrist_ft_motor_force &ob->wrist.ft_motor_force 1}

    {so_u32 wrist_left_enc_channel &rob->wrist.left.enc_channel 1}
    {so_f64 wrist_left_disp &rob->wrist.left.disp 1}
	{so_f64 wrist_left_theta &rob->wrist.left.vel 1}
    {so_f64 wrist_left_torque &rob->wrist.left.torque 1}
    {so_f64 wrist_left_devtrq &rob->wrist.left.devtrq 1}
    {so_f64 wrist_left_xform &rob->wrist.left.xform 1}
    {so_f64 wrist_left_volts &rob->wrist.left.volts 1}
    {so_f64 wrist_left_test_volts &rob->wrist.left.test_volts 1}
    {so_u32 wrist_left_ao_channel &rob->wrist.left.ao_channel 1}
    {so_u32 wrist_left_csen_channel &rob->wrist.left.csen_channel 1}

    {so_u32 wrist_right_enc_channel &rob->wrist.right.enc_channel 1}
    {so_f64 wrist_right_disp &rob->wrist.right.disp 1}
    {so_f64 wrist_right_theta &rob->wrist.right.vel 1}
    {so_f64 wrist_right_torque &rob->wrist.right.torque 1}
    {so_f64 wrist_right_devtrq &rob->wrist.right.devtrq 1}
    {so_f64 wrist_right_xform &rob->wrist.right.xform 1}
    {so_f64 wrist_right_volts &rob->wrist.right.volts 1}
    {so_f64 wrist_right_test_volts &rob->wrist.right.test_volts 1}
    {so_u32 wrist_right_ao_channel &rob->wrist.right.ao_channel 1}
    {so_u32 wrist_right_csen_channel &rob->wrist.right.csen_channel 1}

    {so_u32 wrist_ps_enc_channel &rob->wrist.ps.enc_channel 1}
    {so_f64 wrist_ps_disp &rob->wrist.ps.disp 1}
    {so_f64 wrist_ps_theta &rob->wrist.ps.vel 1}
    {so_f64 wrist_ps_torque &rob->wrist.ps.torque 1}
    {so_f64 wrist_ps_devtrq &rob->wrist.ps.devtrq 1}
    {so_f64 wrist_ps_xform &rob->wrist.ps.xform 1}
    {so_f64 wrist_ps_volts &rob->wrist.ps.volts 1}
    {so_f64 wrist_ps_test_volts &rob->wrist.ps.test_volts 1}
    {so_u32 wrist_ps_ao_channel &rob->wrist.ps.ao_channel 1}
    {so_u32 wrist_ps_csen_channel &rob->wrist.ps.csen_channel 1}

    {so_f64 wrist_diff_gear_ratio &rob->wrist.gears.diff 1}
    {so_f64 wrist_ps_gear_ratio &rob->wrist.gears.ps 1}
    {so_u32 wrist_uei_ao_board_handle &rob->wrist.uei_ao_board_handle 1}

    {so_u32 have_wrist &ob->have_wrist 1}
    {so_f64 wrist_fe_pos &ob->wrist.pos.fe 1}
    {so_f64 wrist_aa_pos &ob->wrist.pos.aa 1}
    {so_f64 wrist_ps_pos &ob->wrist.pos.ps 1}
    {so_f64 wrist_fe_vel &ob->wrist.vel.fe 1}
    {so_f64 wrist_aa_vel &ob->wrist.vel.aa 1}
    {so_f64 wrist_ps_vel &ob->wrist.vel.ps 1}
    {so_f64 wrist_fe_fvel &ob->wrist.fvel.fe 1}
    {so_f64 wrist_aa_fvel &ob->wrist.fvel.aa 1}
    {so_f64 wrist_ps_fvel &ob->wrist.fvel.ps 1}
    {so_f64 wrist_fe_ctltrq &ob->wrist.torque.fe 1}
    {so_f64 wrist_aa_ctltrq &ob->wrist.torque.aa 1}
    {so_f64 wrist_ps_ctltrq &ob->wrist.torque.ps 1}
    {so_f64 wrist_fe_offset &ob->wrist.offset.fe 1}
    {so_f64 wrist_aa_offset &ob->wrist.offset.aa 1}
    {so_f64 wrist_ps_offset &ob->wrist.offset.ps 1}
    {so_f64 wrist_fe_moment_csen &ob->wrist.moment_csen.fe 1}
    {so_f64 wrist_fe_moment_cmd &ob->wrist.moment_cmd.fe 1}
    {so_f64 wrist_aa_moment_csen &ob->wrist.moment_csen.aa 1}
    {so_f64 wrist_aa_moment_cmd &ob->wrist.moment_cmd.aa 1}
    {so_f64 wrist_ps_moment_csen &ob->wrist.moment_csen.ps 1}
    {so_f64 wrist_ps_moment_cmd &ob->wrist.moment_cmd.ps 1}
    {so_f64 wrist_fe_norm &ob->wrist.norm.fe 1}
    {so_f64 wrist_aa_norm &ob->wrist.norm.aa 1}
    {so_f64 wrist_ps_norm &ob->wrist.norm.ps 1}
    {so_f64 wrist_fe_back &ob->wrist.back.fe 1}
    {so_f64 wrist_aa_back &ob->wrist.back.aa 1}
    {so_f64 wrist_ps_back &ob->wrist.back.ps 1}

    {so_f64 ankle_stiff &ob->ankle.stiff 1}
    {so_f64 ankle_damp &ob->ankle.damp 1}
    {so_f64 ankle_rl_pfomax &ob->ankle.rl_pfomax 1}
    {so_f64 ankle_rl_pfotest &ob->ankle.rl_pfotest 1}

    {so_u32 ankle_left_enc_channel &rob->ankle.left.enc_channel 1}
    {so_f64 ankle_left_disp &rob->ankle.left.disp 1}
    {so_f64 ankle_left_devtrq &rob->ankle.left.devtrq 1}
    {so_f64 ankle_left_xform &rob->ankle.left.xform 1}
    {so_f64 ankle_left_volts &rob->ankle.left.volts 1}
    {so_f64 ankle_left_force &rob->ankle.left.force 1}
    {so_f64 ankle_left_test_volts &rob->ankle.left.test_volts 1}
    {so_u32 ankle_left_ao_channel &rob->ankle.left.ao_channel 1}
    {so_u32 ankle_left_csen_channel &rob->ankle.left.csen_channel 1}
    {so_u32 ankle_left_rot_enc_channel &rob->ankle.left.rot_enc_channel 1}
    {so_f64 ankle_left_rot_disp &rob->ankle.left.rot_disp 1}
    {so_f64 ankle_left_rot_lin_disp &rob->ankle.left.rot_lin_disp 1}
    {so_f64 ankle_left_vel &rob->ankle.left.vel 1}
    {so_f64 ankle_left_rot_lin_vel &rob->ankle.left.rot_lin_vel 1}


    {so_u32 ankle_right_enc_channel &rob->ankle.right.enc_channel 1}
    {so_f64 ankle_right_disp &rob->ankle.right.disp 1}
    {so_f64 ankle_right_devtrq &rob->ankle.right.devtrq 1}
    {so_f64 ankle_right_xform &rob->ankle.right.xform 1}
    {so_f64 ankle_right_volts &rob->ankle.right.volts 1}
    {so_f64 ankle_right_force &rob->ankle.right.force 1}
    {so_f64 ankle_right_test_volts &rob->ankle.right.test_volts 1}
    {so_u32 ankle_right_ao_channel &rob->ankle.right.ao_channel 1}
    {so_u32 ankle_right_csen_channel &rob->ankle.right.csen_channel 1}
    {so_u32 ankle_right_rot_enc_channel &rob->ankle.right.rot_enc_channel 1}
    {so_f64 ankle_right_rot_disp &rob->ankle.right.rot_disp 1}
    {so_f64 ankle_right_rot_lin_disp &rob->ankle.right.rot_lin_disp 1}
    {so_f64 ankle_right_vel &rob->ankle.right.vel 1}
    {so_f64 ankle_right_rot_lin_vel &rob->ankle.right.rot_lin_vel 1}

    {so_u32 have_ankle &ob->have_ankle 1}
    {so_f64 ankle_dp_pos &ob->ankle.pos.dp 1}
    {so_f64 ankle_dp_vel &ob->ankle.vel.dp 1}
    {so_f64 ankle_dp_fvel &ob->ankle.fvel.dp 1}
    {so_f64 ankle_dp_accel &ob->ankle.accel.dp 1}
    {so_f64 ankle_dp_torque &ob->ankle.torque.dp 1}
    {so_f64 ankle_dp_moment_csen &ob->ankle.moment_csen.dp 1}
    {so_f64 ankle_dp_moment_cmd &ob->ankle.moment_cmd.dp 1}
    {so_f64 ankle_dp_offset &ob->ankle.offset.dp 1}
    {so_f64 ankle_dp_norm &ob->ankle.norm.dp 1}
    {so_f64 ankle_dp_back &ob->ankle.back.dp 1}
    {so_f64 ankle_ie_pos &ob->ankle.pos.ie 1}
    {so_f64 ankle_ie_vel &ob->ankle.vel.ie 1}
    {so_f64 ankle_ie_fvel &ob->ankle.fvel.ie 1}
    {so_f64 ankle_ie_accel &ob->ankle.accel.ie 1}
    {so_f64 ankle_ie_torque &ob->ankle.torque.ie 1}
    {so_f64 ankle_ie_moment_csen &ob->ankle.moment_csen.ie 1}
    {so_f64 ankle_ie_moment_cmd &ob->ankle.moment_cmd.ie 1}
    {so_f64 ankle_ie_offset &ob->ankle.offset.ie 1}
    {so_f64 ankle_ie_norm &ob->ankle.norm.ie 1}
    {so_f64 ankle_ie_back &ob->ankle.back.ie 1}

    {so_f64 ankle_accel_mag &ob->ankle.accel_mag 1}
    {so_f64 ankle_vel_mag &ob->ankle.vel_mag 1}
    {so_f64 ankle_safety_vel &ob->ankle.safety_vel 1}
    {so_f64 ankle_safety_accel &ob->ankle.safety_accel 1}
    {so_f64 ankle_trans_ratio &rob->ankle.trans.ratio 1}
    {so_f64 ankle_trans_lead &rob->ankle.trans.lead 1}
    {so_f64 ankle_ankle_ball_length &rob->ankle.trans.ankle_ball_length 1}
    {so_f64 ankle_ball_ball_width &rob->ankle.trans.ball_ball_width 1}
    {so_f64 ankle_av_actuator_length &rob->ankle.trans.av_actuator_length 1}
    {so_f64 ankle_av_shin_length &rob->ankle.trans.av_shin_length 1}
    {so_f64 ankle_enc_xform &rob->ankle.trans.enc_xform 1}
    {so_f64 ankle_slip_thresh &rob->ankle.trans.slip_thresh 1}
    {so_u32 ankle_uei_ao_board_handle &rob->ankle.uei_ao_board_handle 1}

    {so_u32 ankle_knee_channel &rob->ankle.knee.channel 1}
    {so_f64 ankle_knee_raw &rob->ankle.knee.raw 1}
    {so_f64 ankle_knee_xform1 &rob->ankle.knee.xform1 1}
    {so_f64 ankle_knee_xform2 &rob->ankle.knee.xform2 1}
    {so_f64 ankle_knee_bias &rob->ankle.knee.bias 1}
    {so_f64 ankle_knee_angle &rob->ankle.knee.angle 1}

    {so_f64 linear_stiff &ob->linear.stiff 1}
    {so_f64 linear_damp &ob->linear.damp 1}
    {so_f64 linear_pfomax &ob->linear.pfomax 1}
    {so_f64 linear_pfotest &ob->linear.pfotest 1}
    {so_u32 linear_adap_going_up &ob->linear.adap_going_up 1}

    {so_u32 linear_enc_channel &rob->linear.motor.enc_channel 1}
    {so_f64 linear_disp &rob->linear.motor.disp 1}
    {so_f64 linear_devfrc &rob->linear.motor.devfrc 1}
    {so_f64 linear_xform &rob->linear.motor.xform 1}
    {so_f64 linear_volts &rob->linear.motor.volts 1}
    {so_f64 linear_test_volts &rob->linear.motor.test_volts 1}
    {so_u32 linear_ao_channel &rob->linear.motor.ao_channel 1}
    {so_f64 linear_limit_volts &rob->linear.motor.limit_volts 1}
    {so_u32 linear_limit_channel &rob->linear.motor.limit_channel 1}

    {so_u32 have_linear &ob->have_linear 1}
    {so_f64 linear_pos &ob->linear.pos 1}
    {so_f64 linear_vel &ob->linear.vel 1}
    {so_f64 linear_fvel &ob->linear.fvel 1}
    {so_f64 linear_force &ob->linear.force 1}
    {so_f64 linear_force_bias &ob->linear.force_bias 1}
    {so_f64 linear_offset &ob->linear.offset 1}
    {so_f64 linear_ref_pos &ob->linear.ref_pos 1}
    {so_f64 linear_back &ob->linear.back 1}
    {so_f64 linear_norm &ob->linear.norm 1}

    {so_f64 linear_gear_ratio &rob->linear.gears.ratio 1}
    {so_u32 linear_uei_ao_board_handle &rob->linear.uei_ao_board_handle 1}

    {so_f64 hand_stiff &ob->hand.stiff 1}
    {so_f64 hand_damp &ob->hand.damp 1}
    {so_f64 hand_pfomax &ob->hand.pfomax 1}
    {so_f64 hand_pfotest &ob->hand.pfotest 1}
    {so_u32 hand_adap_going_up &ob->hand.adap_going_up 1}
    {so_u32 hand_enc_channel &rob->hand.motor.enc_channel 1}
    {so_f64 hand_disp &rob->hand.motor.disp 1}
    {so_f64 hand_devfrc &rob->hand.motor.devfrc 1}
    {so_f64 hand_xform &rob->hand.motor.xform 1}
    {so_f64 hand_volts &rob->hand.motor.volts 1}
    {so_f64 hand_test_volts &rob->hand.motor.test_volts 1}
    {so_u32 hand_ao_channel &rob->hand.motor.ao_channel 1}
    {so_f64 hand_limit_volts &rob->hand.motor.limit_volts 1}
    {so_u32 hand_limit_channel &rob->hand.motor.limit_channel 1}

    {so_u32 have_hand &ob->have_hand 1}
    {so_f64 hand_pos &ob->hand.pos 1}
    {so_f64 hand_vel &ob->hand.vel 1}
    {so_f64 hand_fvel &ob->hand.fvel 1}
    {so_f64 hand_force &ob->hand.force 1}
    {so_f64 hand_grasp &ob->hand.grasp 1}
    {so_f64 hand_force_bias &ob->hand.force_bias 1}
    {so_f64 hand_offset &ob->hand.offset 1}
    {so_f64 hand_ref_pos &ob->hand.ref_pos 1}
    {so_f64 hand_gear_ratio &rob->hand.gears.ratio 1}
    {so_f64 hand_gear_xform &rob->hand.gears.xform 1}
    {so_f64 hand_gear_offset &rob->hand.gears.offset 1}
    {so_u32 hand_uei_ao_board_handle &rob->hand.uei_ao_board_handle 1}

    {so_f64 ain_bias_comp &daq->ain_bias_comp 4}
    {so_u32 ain_cfg &daq->ain_cfg 1}
    {so_s32 ain_got_samples &daq->ain_got_samples 1}
    {so_s32 ain_ret &daq->ain_ret 1}
    {so_s32 n_ueidaq_boards &daq->n_ueidaq_boards 1}
    {so_s32 uei_board &daq->uei_board[0] 4}

    {so_u32 busy &ob->busy 1}
    {so_f64 curl &ob->curl 1}
    {so_f64 damp &ob->damp 1}
    {so_f64 const_force_x &ob->const_force.x 1}
    {so_f64 const_force_y &ob->const_force.y 1}
    {so_s32 debug_level &ob->debug_level 1}
    {so_u32 quit &ob->quit 1}
    {so_u32 doinit &ob->doinit 1}
    {so_u32 didinit &ob->didinit 1}
    {so_u32 butcutoff &ob->butcutoff 1}

    {so_s32 ddfifo &ob->ddfifo 1}
    {so_s32 cififo &ob->cififo 1}
    {so_s32 dififo &ob->dififo 1}
    {so_s32 dofifo &ob->dofifo 1}
    {so_s32 eofifo &ob->eofifo 1}
    {so_u32 fifolen &ob->fifolen 1}
    {so_s32 tcfifo &ob->tcfifo 1}
    {so_u32 ntickfifo &ob->ntickfifo 1}

    {so_f64 s_torque &ob->motor_torque.s 1}
    {so_f64 e_torque &ob->motor_torque.e 1}
    {so_f64 s_volts &ob->motor_volts.s 1}
    {so_f64 e_volts &ob->motor_volts.e 1}
    {so_f64 link_s &rob->link.s 1}
    {so_f64 link_e &rob->link.e 1}

    {so_u32 have_planar &ob->have_planar 1}
    {so_u32 have_tach &ob->have_tach 1}
    {so_u32 have_ft &ob->have_ft 1}
    {so_u32 have_isaft &ob->have_isaft 1}
    {so_u32 have_grasp &ob->have_grasp 1}
    {so_u32 have_accel &ob->have_accel 1}
    {so_u32 have_planar_incenc &ob->have_planar_incenc 1}
    {so_u32 have_planar_ao8 &ob->have_planar_ao8 1}
    {so_u32 Hz &ob->Hz 1}
    {so_u32 i &ob->i 1}
    {so_u32 fasti &ob->fasti 1}
    {so_u32 fastHz &ob->fastHz 1}
    {so_u32 fastirate &ob->fastirate 1}
    {so_f64 fastrate &ob->fastrate 1}
    {so_u32 ovsample &ob->ovsample 1}
    {so_u32 irate &ob->irate 1}
    {so_f64 disp &ob->disp[0] 32}
    {so_u32 ndisp &ob->ndisp 1}
    {so_f64 log &ob->log 32}
    {so_u32 nlog &ob->nlog 1}
    {so_u32 logfnid &ob->logfnid 1}
    {so_f64 refin &ob->refin 32}
    {so_u32 nref &ob->nref 1}
    {so_u32 reffnid &ob->reffnid 1}
    {so_f64 offset_x &rob->offset.x 1}
    {so_f64 offset_y &rob->offset.y 1}
    {so_u32 paused &ob->paused 1}
    {so_u32 fault &ob->fault 1}
    {so_s32 stiffener &ob->stiffener 1}
    {so_s32 stiff_delta &ob->stiff_delta 1}
    {so_u32 no_motors &ob->no_motors 1}
    {so_f64 pi &ob->pi 1}
    {so_f64 pfomax &ob->pfomax 1}
    {so_f64 pfotest &ob->pfotest 1}
    {so_f64 rate &ob->rate 1}
    {so_f64 restart_damp &ob->restart.damp 1}
    {so_u32 restart_go &ob->restart.go 1}
    {so_u32 restart_Hz &ob->restart.Hz 1}
    {so_u32 restart_ovsample &ob->restart.ovsample 1}
    {so_f64 restart_stiff &ob->restart.stiff 1}
    {so_u32 no_safety_check &ob->safety.override 1}
    {so_f64 safety_damping_nms &ob->safety.damping_nms 1}
    {so_f64 safety_pos &ob->safety.pos 1}
    {so_f64 safety_torque &ob->safety.torque 1}
    {so_f64 safety_ramp &ob->safety.ramp 1}
    {so_f64 safety_vel &ob->safety.vel 1}

    {so_u32 safety_planar_just_crossed_back &ob->safety.planar_just_crossed_back 1}
    {so_u32 safety_was_planar_damping &ob->safety.was_planar_damping 1}
    {so_u32 safety_damp_ret_ticks &ob->safety.damp_ret_ticks 1}
    {so_f64 safety_damp_ret_secs &ob->safety.damp_ret_secs 1}

    {so_u32 samplenum &ob->samplenum 1}
    {so_u32 vibrate &ob->vibrate 1}
    {so_s32 xvibe &ob->xvibe 1}
    {so_s32 yvibe &ob->yvibe 1}

    {so_u32 test_raw_torque &ob->test_raw_torque 1}
    {so_f64 raw_torque_volts_s &ob->raw_torque_volts.s 1}
    {so_f64 raw_torque_volts_e &ob->raw_torque_volts.e 1}
    {so_f64 sin_amplitude &ob->sin_amplitude 1}
    {so_f64 sin_period &ob->sin_period 1}
    {so_u32 sin_which_motor &ob->sin_which_motor 1}

    {so_u32 sim_sensors &ob->sim.sensors 1}
    {so_f64 sim_pos_x     &ob->sim.pos.x  1}
    {so_f64 sim_pos_y     &ob->sim.pos.y  1}
    {so_f64 sim_wr_pos_fe &ob->sim.wr_pos.fe 1}
    {so_f64 sim_wr_pos_ps &ob->sim.wr_pos.ps 1}
    {so_f64 sim_wr_pos_aa &ob->sim.wr_pos.aa 1}
    {so_f64 sim_vel_x     &ob->sim.vel.x  1}
    {so_f64 sim_vel_y     &ob->sim.vel.y  1}
    {so_f64 sim_wr_vel_fe &ob->sim.wr_vel.fe 1}
    {so_f64 sim_wr_vel_ps &ob->sim.wr_vel.ps 1}
    {so_f64 sim_wr_vel_aa &ob->sim.wr_vel.aa 1}

    {so_u32 slot_id &ob->copy_slot.id 1}
    {so_u32 slot_i &ob->copy_slot.i 1}
    {so_u32 slot_incr &ob->copy_slot.incr 1}
    {so_u32 slot_term &ob->copy_slot.term 1}
    {so_u32 slot_termi &ob->copy_slot.termi 1}

    {so_f64 slot_b0_x &ob->copy_slot.b0.point.x 1}
    {so_f64 slot_b0_y &ob->copy_slot.b0.point.y 1}
    {so_f64 slot_b0_w &ob->copy_slot.b0.w 1}
    {so_f64 slot_b0_h &ob->copy_slot.b0.h 1}

    {so_f64 slot_b1_x &ob->copy_slot.b1.point.x 1}
    {so_f64 slot_b1_y &ob->copy_slot.b1.point.y 1}
    {so_f64 slot_b1_w &ob->copy_slot.b1.w 1}
    {so_f64 slot_b1_h &ob->copy_slot.b1.h 1}

    {so_u32 slot_fnid &ob->copy_slot.fnid 1}
    {so_f64 slot_rot &ob->copy_slot.rot 1}
    {so_u32 slot_go &ob->copy_slot.go 1}
    {so_u32 slot_running &ob->copy_slot.running 1}
    {so_u32 slot_max &ob->slot_max 1}

    {so_u32 slot0_id &ob->slot[0].id 1}
    {so_u32 slot0_i &ob->slot[0].i 1}
    {so_u32 slot0_incr &ob->slot[0].incr 1}
    {so_u32 slot0_term &ob->slot[0].term 1}
    {so_u32 slot0_termi &ob->slot[0].termi 1}

    {so_f64 slot0_b0_x &ob->slot[0].b0.point.x 1}
    {so_f64 slot0_b0_y &ob->slot[0].b0.point.y 1}
    {so_f64 slot0_b0_w &ob->slot[0].b0.w 1}
    {so_f64 slot0_b0_h &ob->slot[0].b0.h 1}

    {so_f64 slot0_b1_x &ob->slot[0].b1.point.x 1}
    {so_f64 slot0_b1_y &ob->slot[0].b1.point.y 1}
    {so_f64 slot0_b1_w &ob->slot[0].b1.w 1}
    {so_f64 slot0_b1_h &ob->slot[0].b1.h 1}

    {so_f64 slot0_bcur_x &ob->slot[0].bcur.point.x 1}
    {so_f64 slot0_bcur_y &ob->slot[0].bcur.point.y 1}
    {so_f64 slot0_bcur_w &ob->slot[0].bcur.w 1}
    {so_f64 slot0_bcur_h &ob->slot[0].bcur.h 1}

    {so_u32 slot0_fnid &ob->slot[0].fnid 1}
    {so_f64 slot0_rot &ob->slot[0].rot 1}
    {so_u32 slot0_running &ob->slot[0].running 1}
    {so_u32 slot0_go &ob->slot[0].go 1}

    {so_f64 slot1_b0_x &ob->slot[1].b0.point.x 1}
    {so_f64 slot1_b0_y &ob->slot[1].b0.point.y 1}
    {so_f64 slot1_b0_w &ob->slot[1].b0.w 1}
    {so_f64 slot1_b0_h &ob->slot[1].b0.h 1}

    {so_f64 pm_active_power &ob->pm.active_power 1}
    {so_f64 pm_min_jerk_deviation &ob->pm.min_jerk_deviation 1}
    {so_f64 pm_min_jerk_dgraph &ob->pm.min_jerk_dgraph 1}
    {so_f64 pm_dist_straight_line &ob->pm.dist_straight_line 1}
    {so_f64 pm_max_dist_along_axis &ob->pm.max_dist_along_axis 1}
    {so_u32 pm_npoints &ob->pm.npoints 1}
    {so_u32 pm_five_d &ob->pm.five_d 1}

    {so_f64 scr &ob->scr[0] 64}
    {so_f64 scr0 &ob->scr[0] 1}
    {so_f64 scr1 &ob->scr[1] 1}
    {so_f64 scr2 &ob->scr[2] 1}
    {so_f64 scr3 &ob->scr[3] 1}
    {so_f64 scr4 &ob->scr[4] 1}
    {so_f64 scr5 &ob->scr[5] 1}
    {so_f64 scr6 &ob->scr[6] 1}
    {so_f64 scr7 &ob->scr[7] 1}

    {so_f64 game &ob->game[0] 64}

    {so_u16 dienc &daq->m_dienc[0][0] 4}
    {so_s32 dienc_vel &daq->dienc_vel[0] 2}
    {so_s32 dienc_accel &daq->dienc_accel[0] 2}
    {so_u32 diovs &daq->diovs 1}
    {so_u16 dout_buf &daq->m_dout_buf[0] 4}
    {so_u16 dout0 &daq->dout0 1}
    {so_u16 dout1 &daq->dout1 1}
    {so_u16 adc &daq->m_adc[0][0] 64}
    {so_u16 dac &daq->m_dac[0][0] 32}
    {so_f64 adcvolts &daq->m_adcvolts[0][0] 64}
    {so_f64 adcmean &daq->m_adcvoltsmean[0][0] 64}
    {so_f64 adcmed &daq->m_adcvoltsmed[0][0] 64}
    {so_f64 dacvolts &daq->m_dacvolts[0][0] 32}
    {so_u16 distat &daq->distat[0] 2}

    {so_u32 ft_flip &rob->ft.flip 1}
    {so_u32 ft_vert &rob->ft.vert 1}
    {so_u32 ft_dobias &rob->ft.dobias 1}
    {so_f64 ft_offset &rob->ft.offset 1}
    {so_u32 ft_channel &rob->ft.channel[0] 6}
    {so_f64 ft_raw &rob->ft.raw[0] 6}
    {so_f64 ft_curr &rob->ft.curr[0] 6}
    {so_f64 ft_prev &rob->ft.prev[0] 6}
    {so_f64 ft_filt &rob->ft.filt[0] 6}
    {so_f64 ft_sg &rob->ft.sg[0] 6}
    {so_f64 ft_prevf &rob->ft.prevf[0] 6}
    {so_f64 ft_xworld &rob->ft.world.x 1}
    {so_f64 ft_yworld &rob->ft.world.y 1}
    {so_f64 ft_zworld &rob->ft.world.z 1}
    {so_f64 ft_xdev &rob->ft.dev.x 1}
    {so_f64 ft_ydev &rob->ft.dev.y 1}
    {so_f64 ft_zdev &rob->ft.dev.z 1}
    {so_f64 ft_xmoment &rob->ft.moment.x 1}
    {so_f64 ft_ymoment &rob->ft.moment.y 1}
    {so_f64 ft_zmoment &rob->ft.moment.z 1}
    {so_f64 ft_xymag &rob->ft.xymag 1}
    {so_f64 ft_cal &rob->ft.cal[0][0] 36}
    {so_f64 ft_scale &rob->ft.scale[0] 6}
    {so_f64 ft_bias &rob->ft.bias[0] 6}
    {so_f64 ft_cooked &rob->ft.cooked[0] 6}
    {so_f64 ft_avg &rob->ft.avg[0] 6}
    {so_f64 ft_sghist0 &rob->ft.sghist[0][0] 16}

    {so_f64 ft_cal01 &rob->ft.cal[0][1] 1}
    {so_f64 ft_cal10 &rob->ft.cal[1][0] 1}
    {so_f64 ft_cal11 &rob->ft.cal[1][1] 1}

    {so_u16 isaft_cpf &rob->isaft.cpf 1}
    {so_u16 isaft_cpt &rob->isaft.cpt 1}

    {so_s32 isaft_iraw &rob->isaft.iraw 8}
    {so_f64 isaft_raw &rob->isaft.raw 6}

    {so_u32 accel_channel &rob->accel.channel[0] 3}
    {so_f64 accel_raw &rob->accel.raw[0] 3}
    {so_f64 accel_curr &rob->accel.curr[0] 3}
    {so_f64 accel_bias &rob->accel.bias[0] 3}
    {so_f64 accel_xform &rob->accel.xform 1}
    {so_f64 accelx &rob->accel.filt[0] 1}
    {so_f64 accely &rob->accel.filt[1] 1}
    {so_f64 accelz &rob->accel.filt[2] 1}

    {so_u32 have_csen &rob->csen.have 1}
    {so_u32 csen_channela &rob->csen.channela[0] 8}
    {so_u32 csen_channelc &rob->csen.channelc[0] 8}
    {so_f64 csen_rawa &rob->csen.rawa[0] 8}
    {so_f64 csen_rawc &rob->csen.rawc[0] 8}
    {so_f64 csen_xforma &rob->csen.xforma[0] 8}
    {so_f64 csen_xformc &rob->csen.xformc[0] 8}
    {so_f64 csen_kt &rob->csen.kt[0] 8}
    {so_f64 csen_kt_biasa &rob->csen.kt_biasa[0] 8}
    {so_f64 csen_kt_biasc &rob->csen.kt_biasc[0] 8}
    {so_f64 csen_alpha &rob->csen.alpha[0] 8}
    {so_f64 csen_curr &rob->csen.curr[0] 8}
    {so_f64 csen_torque &rob->csen.torque[0] 8}

    {so_u32 grasp_channel &rob->grasp.channel 1}
    {so_f64 grasp_raw &rob->grasp.raw 1}
    {so_f64 grasp_bias &rob->grasp.bias 1}
    {so_f64 grasp_cal &rob->grasp.cal 1}
    {so_f64 grasp_gain &rob->grasp.gain 1}
    {so_f64 grasp_force &rob->grasp.force 1}
    {so_f64 grasp_press &rob->grasp.press 1}
    {so_f64 grasp_release &rob->grasp.release 1}

    {so_f64 isaenc &rob->pc7266.enc 4}
    {so_s32 isaenc_raw &rob->pc7266.raw 4}
    {so_u32 isaenc_max &rob->pc7266.max 1}
    {so_f64 isaenc_scale &rob->pc7266.scale 1}
    {so_u32 isaenc_zero &rob->pc7266.zero 1}
    {so_u32 isaenc_docal &rob->pc7266.docal 1}
    {so_u32 have_pc7266 &rob->pc7266.have 1}

    {so_u32 have_pci4e &rob->pci4e.have 1}
    {so_u32 pci4e_bar &rob->pci4e.bar 1}
    {so_u32 pci4e_remap &rob->pci4e.remap 1}
    {so_u32 pci4e_len &rob->pci4e.len 1}
    {so_u32 pci4e_dev &rob->pci4e.dev 1}

    {so_f64 pcienc &rob->pci4e.enc 4}
    {so_f64 pcienc_lin &rob->pci4e.lenc 4}
    {so_u32 pcienc_raw &rob->pci4e.raw 4}
    {so_u32 pcienc_limit &rob->pci4e.limit 1}
    {so_f64 pcienc_scale &rob->pci4e.scale 1}
    {so_u32 pcienc_zero &rob->pci4e.zero 1}

    {so_f64 stiff &ob->stiff 1}
    {so_f64 side_stiff &ob->side_stiff 1}
    {so_u8 tag &ob->tag[0] 8}
    {so_u32 ticks30Hz &ob->ticks30Hz 1}

    {so_u64 time_after_last_sample &ob->times.time_after_last_sample 1}
    {so_u64 time_after_sample &ob->times.time_after_sample 1}
    {so_u64 time_at_start &ob->times.time_at_start 1}
    {so_u64 time_before_last_sample &ob->times.time_before_last_sample 1}
    {so_u64 time_before_sample &ob->times.time_before_sample 1}
    {so_u64 time_delta_call &ob->times.time_delta_call 1}
    {so_u64 time_delta_sample &ob->times.time_delta_sample 1}
    {so_u64 time_delta_tick &ob->times.time_delta_tick 1}
    {so_u64 time_since_start &ob->times.time_since_start 1}
    {so_u32 time_ns_delta_call &ob->times.ns_delta_call 1}
    {so_u32 time_ns_delta_sample &ob->times.ns_delta_sample 1}
    {so_u32 time_ns_delta_tick &ob->times.ns_delta_tick 1}
    {so_u32 time_ms_since_start &ob->times.ms_since_start 1}
    {so_u32 sec &ob->times.sec 1}
    {so_u32 time_ns_delta_sample_thresh &ob->times.ns_delta_sample_thresh 1}
    {so_u32 time_ns_delta_tick_thresh &ob->times.ns_delta_tick_thresh 1}
    {so_u32 time_ns_max_delta_sample &ob->times.ns_max_delta_sample 1}
    {so_u32 time_ns_max_delta_tick &ob->times.ns_max_delta_tick 1}

    {so_u32 total_samples &ob->total_samples 1}
    {so_f64 x &ob->pos.x 1}
    {so_f64 y &ob->pos.y 1}
    {so_f64 x_force &ob->motor_force.x 1}
    {so_f64 y_force &ob->motor_force.y 1}
    {so_f64 soft_xvel &ob->soft_vel.x 1}
    {so_f64 soft_yvel &ob->soft_vel.y 1}
    {so_f64 fsoft_xvel &ob->fsoft_vel.x 1}
    {so_f64 fsoft_yvel &ob->fsoft_vel.y 1}
    {so_f64 tach_xvel &ob->tach_vel.x 1}
    {so_f64 tach_yvel &ob->tach_vel.y 1}
    {so_f64 xvel &ob->vel.x 1}
    {so_f64 yvel &ob->vel.y 1}
    {so_f64 velmag &ob->velmag 1}
    {so_f64 soft_accelx &ob->soft_accel.x 1}
    {so_f64 soft_accely &ob->soft_accel.y 1}
    {so_f64 soft_accelmag &ob->soft_accelmag 1}
    {so_f64 stheta &ob->theta.s 1}
    {so_f64 etheta &ob->theta.e 1}
    {so_f64 sthetadot &ob->thetadot.s 1}
    {so_f64 ethetadot &ob->thetadot.e 1}
    {so_f64 pl_back_x &ob->back.x 1}
    {so_f64 pl_back_y &ob->back.y 1}
    {so_f64 pl_norm_x &ob->norm.x 1}
    {so_f64 pl_norm_y &ob->norm.y 1}

    {so_u32 errnum &ob->errnum 1}
    {so_u32 nerrors &ob->nerrors 1}
    {so_u32 errori &ob->errori[0] 128}
    {so_u32 errorcode &ob->errorcode[0] 128}
    {so_u32 errorindex &ob->errorindex 1}

    {so_u32 g_go &game->go 1}

    {so_f64 g_planet_color_red      &game->planet_color_red[0]      9 }
    {so_f64 g_planet_color_green    &game->planet_color_green[0]    9 }
    {so_f64 g_planet_color_blue     &game->planet_color_blue[0]     9 }
    {so_u32 g_planet_color_go       &game->planet_color_go[0]       9 }
    {so_f64 g_planet_default_red    &game->planet_default_red     1 }
    {so_f64 g_planet_default_green  &game->planet_default_green   1 }
    {so_f64 g_planet_default_blue   &game->planet_default_blue    1 }
    {so_f64 g_planet_rotation_aa    &game->planet_rotation_aa[0]    9 }
    {so_f64 g_planet_rotation_ps    &game->planet_rotation_ps[0]    9 }
    {so_f64 g_planet_rotation_fe    &game->planet_rotation_fe[0]    9 }
    {so_u32 g_planet_rotation_go    &game->planet_rotation_go[0]    9 }

    {so_u32 g_target                &game->target                   1 }
    {so_u32 g_target_go             &game->target_go                1 }
    {so_u32 g_target_hit            &game->target_hit               1 }

    {so_u32 g_repetition            &game->repetition               1 }
    {so_u32 g_total_repetitions     &game->total_repetitions        1 }
    {so_u32 g_repetition_go         &game->repetition_go            1 }    

    {so_u32 g_key_spacebar          &game->key_spacebar             1 }
    {so_u32 g_key_quit              &game->key_quit                 1 }
    {so_u32 g_key_b                 &game->key_b                    1 }

    {so_f64 g_feSlack               &game->feSlack                  1 }
    {so_f64 g_psSlack               &game->psSlack                  1 }
    {so_f64 g_aaSlack               &game->aaSlack                  1 }
    {so_f64 g_planarSlack           &game->planarSlack              1 }
    {so_u32 g_slack_go              &game->slack_go                 1 }

    {so_u32 g_active_target         &game->active_target            1 }
    {so_u32 g_active_target_go      &game->active_target_go         1 }

    {so_u32 g_quit_server           &game->quit_server              1 }

    {so_f64 scorriolis &dyncmp_var->Corriolis.s 1}
    {so_f64 ecorriolis &dyncmp_var->Corriolis.e 1}
    {so_f64 mass11 &dyncmp_var->Mofq.e00 1}
    {so_f64 mass12 &dyncmp_var->Mofq.e01 1}
    {so_f64 mass21 &dyncmp_var->Mofq.e10 1}
    {so_f64 mass22 &dyncmp_var->Mofq.e11 1}


    {so_u32 pixelx &moh->pixelx[0] 36}
    {so_u32 pixely &moh->pixely[0] 36}
    {so_f64 realx &moh->realx[0] 36}
    {so_f64 realy &moh->realy[0] 36}	
    {so_f64 xcenter &moh->xcenter 1}
    {so_f64 ycenter &moh->ycenter 1}
    {so_f64 ffs &moh->ffs 1}
    {so_f64 hand_place &moh->hand_place 1}
    {so_f64 stiffness &moh->stiffness 1}
    {so_f64 move_flag &moh->move_flag 1}
    {so_f64 servo_flag &moh->servo_flag 1}
    {so_f64 current_dir &moh->current_dir 1}
    {so_f64 last_pointX &moh->last_pointX 1}
    {so_f64 last_pointY &moh->last_pointY 1}

    {so_f64 plg_debug &ob->plg_debug 1}
    {so_f64 plg_ftzero &ob->plg_ftzero[0] 6}
    {so_u32 plg_ftzerocount &ob->plg_ftzerocount 1}
    {so_f64 plg_curlmat &ob->plg_curlmat[0] 4}
    {so_f64 plg_stiffness &ob->plg_stiffness 1}
    {so_f64 plg_damping &ob->plg_damping 1}
    {so_f64 plg_p1x &ob->plg_p1x 1}
    {so_f64 plg_p1y &ob->plg_p1y 1}
    {so_f64 plg_p2x &ob->plg_p2x 1}
    {so_f64 plg_p2y &ob->plg_p2y 1}
    {so_f64 plg_movetime &ob->plg_movetime 1}
    {so_u32 plg_counterstart &ob->plg_counterstart 1}
    {so_u32 plg_moveto_done &ob->plg_moveto_done 1}
    {so_f64 plg_last_fX &ob->plg_last_fX 1}
    {so_f64 plg_last_fY &ob->plg_last_fY 1}
    {so_f64 plg_channel_width &ob->plg_channel_width 1}
    
    {so_u32 ba_accumforce &ob->ba_accumforce 1}
    {so_f64 mkt_mvtangle &ob->mkt_mvtangle 1}
    {so_f64 mkt_finline &ob->mkt_finline 1}
    {so_f64 mkt_fortho &ob->mkt_fortho 1}
    
    {so_f64 ba_fx &ob->ba_fx 1}
    {so_f64 ba_fy &ob->ba_fy 1}
    {so_f64 ba_position &ob->ba_position 1}

    {so_u32 mkt_isMcGill    &ob->mkt_isMcGill 1}
    {so_u32 fvv_trial_phase &ob->fvv_trial_phase 1}
    {so_u32 fvv_trial_no    &ob->fvv_trial_no 1}

    {so_u32 last_shm_val &ob->last_shm_val 1}

}
    # last_shm_val must be last.

# TODO: delete    {so_s32 nfifos &ob->nfifos 1}, was after ntickfifo
