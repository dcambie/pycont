import time
import numpy as np
import logging

log = logging.getLogger("SerialLabware")
log = logging.getLogger("pycont")
log.setLevel(logging.DEBUG)
ch = logging.StreamHandler()
formatter = logging.Formatter("%(asctime)s - %(name)s::%(levelname)s -- %(message)s")
ch.setFormatter(formatter)
log.addHandler(ch)


import pycont.controller
SETUP_CONFIG_FILE = './pump_setup_config.json'
controller = pycont.controller.MultiPumpController.from_configfile(SETUP_CONFIG_FILE)


t = []
for _ in range(10):
	start_time = time.time()
	controller.smart_initialize()
	t.append(time.time()-start_time)

t = np.array(t)
print(f"Initialization time (min/max/avg+-st.dev.) -  {min(t)}/{max(t)}/{np.average(t)}+-{np.std(t)}")

### Pycont optimizied settings w/ readline implemented in SL2 ###

#     # Serial
#     parameters = {
#         "baudrate": baudrate,
#         "transmit_timeout": 0,
#         "receive_timeout": timeout,
#         "receiving_interval": 0.01,
#         "port": port,
#         "command_delay": 0,
#         "readline": True
#     }

# Initialization time (min/max/avg+-st.dev.) -  5.610002756118774/5.849605083465576/5.783875823020935+-0.06887113146877084


### Pycont optimizied settings w/ std SL2 ###

#     # Serial
#     parameters = {
#         "baudrate": baudrate,
#         "transmit_timeout": 0,
#         "receive_timeout": timeout,
#         "receiving_interval": 0.01,
#         "port": port,
#         "command_delay": 0,
#     }

# Initialization time (min/max/avg+-st.dev.) -  19.113630294799805/19.301772832870483/19.187334418296814+-0.06184453419343627


### Default SL2 SerialConnection settings ###

#     # Serial
#     parameters = {
#         "baudrate": baudrate,
#         "port": port,
#     }

# Initialization time (min/max/avg+-st.dev.) -  108.9447569847107/109.50227332115173/109.1000191450119+-0.15917385843150753