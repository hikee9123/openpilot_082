import json
import os


json_file_name = '/data/openpilot/atom_2.json'

class kegman_conf():
  def __init__(self, CP=None):
    self.config = None
    self.init = { 
        "tun_type": "lqr",
        "learnerParams": 1, 
        "ap_autoReasume": 1,
        "ap_autoScnOffTime": 10,
        "cv_KPH": [30,60],
        "cv_BPV": [[255],[100,255]],
        "cv_sMaxV": [[384],[384,384]],
        "cv_sdDNV": [[1],[5,1]],
        "cv_sdUPV": [[1],[3,1]],
        "sR_KPH": [30,60],
        "sR_BPV": [[255],[100,255]],
        "sR_lqr_kiV": [[0.01],[0.02,0.01]],
        "sR_lqr_scaleV": [[2000],[1850,1900]],
        "sR_pid_KpV": [[0.2],[0.20,0.15]],
        "sR_pid_KiV": [[0.1],[0.02,0.02]],
        "sR_pid_KdV": [[2.5],[2.5,2.5]],
        "sR_steerRatioV": [[13.2],[17.3,15.0]],
        "sR_ActuatorDelayV": [[0.1],[0.1,0.2]],
        "sR_pid_deadzone": 0.0,
        "steerLimitTimer": 0.8,
        "steerOffset": 0.0,
        "steerRateCost": 1.0,
        "cameraOffset": 0.00
         }


  def data_check(self, name, value ):
    if name not in self.config:
        self.config.update({name:value})
        self.element_updated = True


  def read_config(self):
    self.element_updated = False

    if os.path.isfile( json_file_name ):
      with open( json_file_name, 'r') as f:
        str_kegman = f.read()
        print( str_kegman )
        self.config = json.loads(str_kegman)

      for name in self.init:
        self.data_check( name, self.init[name] )

      if self.element_updated:
        print("updated")
        self.write_config(self.config)

    else:
      self.config = self.init      
      self.write_config(self.config)

    return self.config

  def write_config(self, config):
    try:
      with open( json_file_name, 'w') as f:
        json.dump(self.config, f, indent=2, sort_keys=True)
        os.chmod( json_file_name, 0o764)
    except IOError:
      os.mkdir('/data')
      with open( json_file_name, 'w') as f:
        json.dump(self.config, f, indent=2, sort_keys=True)
        os.chmod( json_file_name, 0o764)
