import os
import math
from common.numpy_fast import interp

import cereal.messaging as messaging
from selfdrive.swaglog import cloudlog
from common.realtime import sec_since_boot
from selfdrive.controls.lib.radar_helpers import _LEAD_ACCEL_TAU
from selfdrive.controls.lib.longitudinal_mpc import libmpc_py
from selfdrive.controls.lib.drive_helpers import MPC_COST_LONG

LOG_MPC = os.environ.get('LOG_MPC', False)

## Variables that change braking profiles

# In seconds
ONE_BAR_DISTANCE = 0.9
TWO_BAR_DISTANCE = 1.3
THREE_BAR_DISTANCE = 1.8
STOPPING_DISTANCE = 2  # distance between you and lead car when you come to stop
TR = TWO_BAR_DISTANCE  # default interval

# braking profile changes when below this speed based on following dynamics below [m/s]
CITY_SPEED = 19.44

ONE_BAR_BP_0 = -0.1
ONE_BAR_BP_1 = 2.25
TWO_BAR_BP_0 = -0.1
TWO_BAR_BP_1 = 2.5
THREE_BAR_BP_0 = 0.0
THREE_BAR_BP_1 = 3.0

ONE_BAR_MAX = 2.1
TWO_BAR_MAX = 2.1
THREE_BAR_MAX = 2.1

ONE_BAR_HWY = 0.4
TWO_BAR_HWY = 0.3
THREE_BAR_HWY = 0.1

# City braking profile changes (makes the car brake harder because it wants to be farther from the lead car - increase to brake harder)
ONE_BAR_PROFILE = [ONE_BAR_DISTANCE, ONE_BAR_MAX]
ONE_BAR_PROFILE_BP = [ONE_BAR_BP_0, ONE_BAR_BP_1]

TWO_BAR_PROFILE = [TWO_BAR_DISTANCE, TWO_BAR_MAX]
TWO_BAR_PROFILE_BP = [TWO_BAR_BP_0, TWO_BAR_BP_1]

THREE_BAR_PROFILE = [THREE_BAR_DISTANCE, THREE_BAR_MAX]
THREE_BAR_PROFILE_BP = [THREE_BAR_BP_0, THREE_BAR_BP_1]

# Highway braking profiles
H_ONE_BAR_PROFILE = [ONE_BAR_DISTANCE, ONE_BAR_DISTANCE + ONE_BAR_HWY]
H_ONE_BAR_PROFILE_BP = [0.0, 2.5]

H_TWO_BAR_PROFILE = [TWO_BAR_DISTANCE, TWO_BAR_DISTANCE + TWO_BAR_HWY]
H_TWO_BAR_PROFILE_BP = [0.0, 3.0]

H_THREE_BAR_PROFILE = [THREE_BAR_DISTANCE, THREE_BAR_DISTANCE + THREE_BAR_HWY]
H_THREE_BAR_PROFILE_BP = [0.0, 4.0]


class LongitudinalMpc():
  def __init__(self, mpc_id):
    self.mpc_id = mpc_id

    self.setup_mpc()
    self.v_mpc = 0.0
    self.v_mpc_future = 0.0
    self.a_mpc = 0.0
    self.v_cruise = 0.0
    self.prev_lead_status = False
    self.prev_lead_x = 0.0
    self.new_lead = False

    self.last_cloudlog_t = 0.0

    self.v_rel = 10.0
    self.lastTR = 2

  def send_mpc_solution(self, pm, qp_iterations, calculation_time):
    qp_iterations = max(0, qp_iterations)
    dat = messaging.new_message('liveLongitudinalMpc')
    dat.liveLongitudinalMpc.xEgo = list(self.mpc_solution[0].x_ego)
    dat.liveLongitudinalMpc.vEgo = list(self.mpc_solution[0].v_ego)
    dat.liveLongitudinalMpc.aEgo = list(self.mpc_solution[0].a_ego)
    dat.liveLongitudinalMpc.xLead = list(self.mpc_solution[0].x_l)
    dat.liveLongitudinalMpc.vLead = list(self.mpc_solution[0].v_l)
    dat.liveLongitudinalMpc.cost = self.mpc_solution[0].cost
    dat.liveLongitudinalMpc.aLeadTau = self.a_lead_tau
    dat.liveLongitudinalMpc.qpIterations = qp_iterations
    dat.liveLongitudinalMpc.mpcId = self.mpc_id
    dat.liveLongitudinalMpc.calculationTime = calculation_time
    pm.send('liveLongitudinalMpc', dat)

  def setup_mpc(self):
    ffi, self.libmpc = libmpc_py.get_libmpc(self.mpc_id)
    self.libmpc.init(MPC_COST_LONG.TTC, MPC_COST_LONG.DISTANCE,
                     MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)

    self.mpc_solution = ffi.new("log_t *")
    self.cur_state = ffi.new("state_t *")
    self.cur_state[0].v_ego = 0
    self.cur_state[0].a_ego = 0
    self.a_lead_tau = _LEAD_ACCEL_TAU

  def set_cur_state(self, v, a):
    self.cur_state[0].v_ego = v
    self.cur_state[0].a_ego = a

  def update(self, pm, CS, lead, v_cruise_setpoint):
    v_ego = CS.vEgo

    # Setup current mpc state
    self.cur_state[0].x_ego = 0.0

    if lead is not None and lead.status:
      # x_lead = lead.dRel
      x_lead = max(0, lead.dRel - STOPPING_DISTANCE)  # increase stopping distance to car by X [m]
      v_lead = max(0.0, lead.vLead)
      a_lead = lead.aLeadK

      if (v_lead < 0.1 or -a_lead / 2.0 > v_lead):
        v_lead = 0.0
        a_lead = 0.0

      # self.a_lead_tau = lead.aLeadTau
      self.a_lead_tau = max(lead.aLeadTau, (a_lead ** 2 * math.pi) / (2 * (v_lead + 0.01) ** 2))
      self.new_lead = False
      if not self.prev_lead_status or abs(x_lead - self.prev_lead_x) > 2.5:
        self.libmpc.init_with_simulation(self.v_mpc, x_lead, v_lead, a_lead, self.a_lead_tau)
        self.new_lead = True

      self.prev_lead_status = True
      self.prev_lead_x = x_lead
      self.cur_state[0].x_l = x_lead
      self.cur_state[0].v_l = v_lead
    else:
      self.prev_lead_status = False
      # Fake a fast lead car, so mpc keeps running
      self.cur_state[0].x_l = 50.0
      self.cur_state[0].v_l = v_ego + 10.0
      a_lead = 0.0
      v_lead = 0.0
      self.a_lead_tau = _LEAD_ACCEL_TAU

    # Calculate conditions
    self.v_rel = v_lead - v_ego   # calculate relative velocity vs lead car

    # Is the car running surface street speeds?
    if v_ego < CITY_SPEED:
      self.street_speed = 1
    else:
      self.street_speed = 0

    # Calculate mpc
    # Adjust distance from lead car when distance button pressed
    if CS.readdistancelines == 1:
      if self.street_speed:
        TR = interp(-self.v_rel, ONE_BAR_PROFILE_BP, ONE_BAR_PROFILE)
      else:
        TR = interp(-self.v_rel, H_ONE_BAR_PROFILE_BP, H_ONE_BAR_PROFILE)
      if CS.readdistancelines != self.lastTR:
        self.libmpc.init(MPC_COST_LONG.TTC, 1.0, MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)
        self.lastTR = CS.readdistancelines
    elif CS.readdistancelines == 2:
      if self.street_speed:
        TR = interp(-self.v_rel, TWO_BAR_PROFILE_BP, TWO_BAR_PROFILE)
      else:
        TR = interp(-self.v_rel, H_TWO_BAR_PROFILE_BP, H_TWO_BAR_PROFILE)
      if CS.readdistancelines != self.lastTR:
        self.libmpc.init(MPC_COST_LONG.TTC, MPC_COST_LONG.DISTANCE, MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)
        self.lastTR = CS.readdistancelines
    elif CS.readdistancelines == 3:
      if self.street_speed:
        TR = interp(-self.v_rel, THREE_BAR_PROFILE_BP, THREE_BAR_PROFILE)
      else:
        TR = interp(-self.v_rel, H_THREE_BAR_PROFILE_BP, H_THREE_BAR_PROFILE)
      if CS.readdistancelines != self.lastTR:
        self.libmpc.init(MPC_COST_LONG.TTC, MPC_COST_LONG.DISTANCE, MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)
        self.lastTR = CS.readdistancelines
    # if readdistancelines != 1,2,3
    else:
     TR = TWO_BAR_DISTANCE
     self.libmpc.init(MPC_COST_LONG.TTC, MPC_COST_LONG.DISTANCE, MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)

    t = sec_since_boot()
    # n_its = self.libmpc.run_mpc(self.cur_state, self.mpc_solution, self.a_lead_tau, a_lead)
    n_its = self.libmpc.run_mpc(self.cur_state, self.mpc_solution, self.a_lead_tau, a_lead, TR)
    duration = int((sec_since_boot() - t) * 1e9)

    if LOG_MPC:
      self.send_mpc_solution(pm, n_its, duration)

    # Get solution. MPC timestep is 0.2 s, so interpolation to 0.05 s is needed
    self.v_mpc = self.mpc_solution[0].v_ego[1]
    self.a_mpc = self.mpc_solution[0].a_ego[1]
    self.v_mpc_future = self.mpc_solution[0].v_ego[10]

    # Reset if NaN or goes through lead car
    crashing = any(lead - ego < -50 for (lead, ego) in zip(self.mpc_solution[0].x_l, self.mpc_solution[0].x_ego))
    nans = any(math.isnan(x) for x in self.mpc_solution[0].v_ego)
    backwards = min(self.mpc_solution[0].v_ego) < -0.01

    if ((backwards or crashing) and self.prev_lead_status) or nans:
      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning("Longitudinal mpc %d reset - backwards: %s crashing: %s nan: %s" % (
                          self.mpc_id, backwards, crashing, nans))

      self.libmpc.init(MPC_COST_LONG.TTC, MPC_COST_LONG.DISTANCE,
                       MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)
      self.cur_state[0].v_ego = v_ego
      self.cur_state[0].a_ego = 0.0
      self.v_mpc = v_ego
      self.a_mpc = CS.aEgo
      self.prev_lead_status = False
