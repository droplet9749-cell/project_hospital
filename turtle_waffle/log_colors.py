# ~robot_ws/src/project_hospital/turtle_waffle/log_colors.py

class Color:
    RESET = '\033[0m'
    NAV = '\033[36m'      # Cyan (청록색)
    ARM = '\033[35m'      # Magenta (자주색)
    SERVER = '\033[34m'   # Blue (파란색)
    TASK = '\033[32m'     # Green (초록색)
    WARN = '\033[33m'     # Yellow (노란색)
    ERR = '\033[31m'      # Red (빨간색)

class Tag:
    # 모듈별 기본 태그
    NAV = f"{Color.NAV}[NAV]{Color.RESET}"
    ARM = f"{Color.ARM}[ARM]{Color.RESET}"
    SRV = f"{Color.SERVER}[SERVER]{Color.RESET}"
    TSK = f"{Color.TASK}[TASK]{Color.RESET}"
    
    # 상태별 태그
    OK = f"{Color.TASK}[ OK ]{Color.RESET}"
    FAIL = f"{Color.ERR}[FAIL]{Color.RESET}"
    WARN = f"{Color.WARN}[WARN]{Color.RESET}"