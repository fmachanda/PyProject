import os
import asyncio
import logging

os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil
m = mavutil.mavlink


class GlobalState:
    CUSTOM_MODE_UNINIT = 0
    CUSTOM_MODE_BOOT = 1
    CUSTOM_MODE_GROUND = 2
    CUSTOM_MODE_TAKEOFF = 3
    CUSTOM_MODE_FLIGHT = 4
    CUSTOM_MODE_LANDING = 5

    CUSTOM_SUBMODE_UNINIT = 0
    CUSTOM_SUBMODE_BOOT = 10
    CUSTOM_SUBMODE_SHUTDOWN = 11
    CUSTOM_SUBMODE_GROUND_DISARMED = 20
    CUSTOM_SUBMODE_GROUND_ARMED = 21
    CUSTOM_SUBMODE_TAKEOFF_ASCENT = 30
    CUSTOM_SUBMODE_TAKEOFF_HOVER = 31
    CUSTOM_SUBMODE_TAKEOFF_TRANSIT = 32
    CUSTOM_SUBMODE_FLIGHT_NORMAL = 40
    CUSTOM_SUBMODE_FLIGHT_MANUAL = 41
    CUSTOM_SUBMODE_FLIGHT_TERRAIN_AVOIDANCE = 42
    CUSTOM_SUBMODE_LANDING_TRANSIT = 50
    CUSTOM_SUBMODE_LANDING_HOVER = 51
    CUSTOM_SUBMODE_LANDING_DESCENT = 52

    CUSTOM_MODE_NAMES = {
        CUSTOM_MODE_UNINIT: 'UNINIT',
        CUSTOM_MODE_BOOT: 'BOOT',
        CUSTOM_MODE_GROUND: 'GROUND',
        CUSTOM_MODE_TAKEOFF: 'TAKEOFF',
        CUSTOM_MODE_FLIGHT: 'FLIGHT',
        CUSTOM_MODE_LANDING: 'LANDING',
    }

    CUSTOM_SUBMODE_NAMES = {
        CUSTOM_SUBMODE_UNINIT: 'UNINIT',
        CUSTOM_SUBMODE_BOOT: 'BOOT',
        CUSTOM_SUBMODE_SHUTDOWN: 'SHUTDOWN',
        CUSTOM_SUBMODE_GROUND_DISARMED: 'GROUND_DISARMED',
        CUSTOM_SUBMODE_GROUND_ARMED: 'GROUND_ARMED',
        CUSTOM_SUBMODE_TAKEOFF_ASCENT: 'TAKEOFF_ASCENT',
        CUSTOM_SUBMODE_TAKEOFF_HOVER: 'TAKEOFF_HOVER',
        CUSTOM_SUBMODE_TAKEOFF_TRANSIT: 'TAKEOFF_TRANSIT',
        CUSTOM_SUBMODE_FLIGHT_NORMAL: 'FLIGHT_NORMAL',
        CUSTOM_SUBMODE_FLIGHT_MANUAL: 'FLIGHT_MANUAL',
        CUSTOM_SUBMODE_FLIGHT_TERRAIN_AVOIDANCE: 'FLIGHT_TERRAIN_AVOIDANCE',
        CUSTOM_SUBMODE_LANDING_TRANSIT: 'LANDING_TRANSIT',
        CUSTOM_SUBMODE_LANDING_HOVER: 'LANDING_HOVER',
        CUSTOM_SUBMODE_LANDING_DESCENT: 'LANDING_DESCENT',
    }

    MAV_MODES = [
        m.MAV_MODE_PREFLIGHT,
        m.MAV_MODE_MANUAL_ARMED,
        # m.MAV_MODE_MANUAL_DISARMED,
        m.MAV_MODE_GUIDED_ARMED,
        # m.MAV_MODE_GUIDED_DISARMED,
        m.MAV_MODE_AUTO_ARMED,
        m.MAV_MODE_AUTO_DISARMED,
    ]

    MAV_STATES_NOMINAL = [
        m.MAV_STATE_UNINIT,
        m.MAV_STATE_BOOT,
        m.MAV_STATE_CALIBRATING,
        m.MAV_STATE_STANDBY,
        m.MAV_STATE_ACTIVE,
    ]

    MAV_STATES_ABNORMAL = [
        m.MAV_STATE_CRITICAL,
        m.MAV_STATE_EMERGENCY,
        m.MAV_STATE_POWEROFF,
        # m.MAV_STATE_FLIGHT_TERMINATION,
    ]

    CUSTOM_MODES = [
        CUSTOM_MODE_UNINIT,
        CUSTOM_MODE_BOOT,
        CUSTOM_MODE_GROUND,
        CUSTOM_MODE_TAKEOFF,
        CUSTOM_MODE_FLIGHT,
        CUSTOM_MODE_LANDING,
    ]

    CUSTOM_SUBMODES = [
        CUSTOM_SUBMODE_UNINIT,
        CUSTOM_SUBMODE_BOOT,
        CUSTOM_SUBMODE_SHUTDOWN,
        CUSTOM_SUBMODE_GROUND_DISARMED,
        CUSTOM_SUBMODE_GROUND_ARMED,
        CUSTOM_SUBMODE_TAKEOFF_ASCENT,
        CUSTOM_SUBMODE_TAKEOFF_HOVER,
        CUSTOM_SUBMODE_TAKEOFF_TRANSIT,
        CUSTOM_SUBMODE_FLIGHT_NORMAL,
        CUSTOM_SUBMODE_FLIGHT_MANUAL,
        CUSTOM_SUBMODE_FLIGHT_TERRAIN_AVOIDANCE,
        CUSTOM_SUBMODE_LANDING_TRANSIT,
        CUSTOM_SUBMODE_LANDING_HOVER,
        CUSTOM_SUBMODE_LANDING_DESCENT,
    ]

    # Index 0 is default for that submode
    ALLOWED_CUSTOM_MODES = {
        CUSTOM_SUBMODE_UNINIT:[CUSTOM_MODE_UNINIT],
        CUSTOM_SUBMODE_BOOT:[CUSTOM_MODE_BOOT],
        CUSTOM_SUBMODE_SHUTDOWN:[CUSTOM_MODE_BOOT],
        CUSTOM_SUBMODE_GROUND_DISARMED:[CUSTOM_MODE_GROUND],
        CUSTOM_SUBMODE_GROUND_ARMED:[CUSTOM_MODE_GROUND],
        CUSTOM_SUBMODE_TAKEOFF_ASCENT:[CUSTOM_MODE_TAKEOFF],
        CUSTOM_SUBMODE_TAKEOFF_HOVER:[CUSTOM_MODE_TAKEOFF],
        CUSTOM_SUBMODE_TAKEOFF_TRANSIT:[CUSTOM_MODE_TAKEOFF],
        CUSTOM_SUBMODE_FLIGHT_NORMAL:[CUSTOM_MODE_FLIGHT],
        CUSTOM_SUBMODE_FLIGHT_MANUAL:[CUSTOM_MODE_FLIGHT],
        CUSTOM_SUBMODE_FLIGHT_TERRAIN_AVOIDANCE:[CUSTOM_MODE_FLIGHT],
        CUSTOM_SUBMODE_LANDING_TRANSIT:[CUSTOM_MODE_LANDING],
        CUSTOM_SUBMODE_LANDING_HOVER:[CUSTOM_MODE_LANDING],
        CUSTOM_SUBMODE_LANDING_DESCENT:[CUSTOM_MODE_LANDING],
    }

    ALLOWED_MODES = {
        CUSTOM_SUBMODE_UNINIT:[m.MAV_MODE_PREFLIGHT],
        CUSTOM_SUBMODE_BOOT:[m.MAV_MODE_PREFLIGHT],
        CUSTOM_SUBMODE_SHUTDOWN:[m.MAV_MODE_PREFLIGHT],
        CUSTOM_SUBMODE_GROUND_DISARMED:[m.MAV_MODE_PREFLIGHT, m.MAV_MODE_AUTO_DISARMED],
        CUSTOM_SUBMODE_GROUND_ARMED:[m.MAV_MODE_GUIDED_ARMED, m.MAV_MODE_AUTO_ARMED],
        CUSTOM_SUBMODE_TAKEOFF_ASCENT:[m.MAV_MODE_GUIDED_ARMED, m.MAV_MODE_AUTO_ARMED],
        CUSTOM_SUBMODE_TAKEOFF_HOVER:[m.MAV_MODE_GUIDED_ARMED, m.MAV_MODE_AUTO_ARMED],
        CUSTOM_SUBMODE_TAKEOFF_TRANSIT:[m.MAV_MODE_GUIDED_ARMED, m.MAV_MODE_AUTO_ARMED],
        CUSTOM_SUBMODE_FLIGHT_NORMAL:[m.MAV_MODE_GUIDED_ARMED, m.MAV_MODE_AUTO_ARMED],
        CUSTOM_SUBMODE_FLIGHT_MANUAL:[m.MAV_MODE_MANUAL_ARMED],
        CUSTOM_SUBMODE_FLIGHT_TERRAIN_AVOIDANCE:[m.MAV_MODE_GUIDED_ARMED, m.MAV_MODE_AUTO_ARMED],
        CUSTOM_SUBMODE_LANDING_TRANSIT:[m.MAV_MODE_GUIDED_ARMED, m.MAV_MODE_AUTO_ARMED],
        CUSTOM_SUBMODE_LANDING_HOVER:[m.MAV_MODE_GUIDED_ARMED, m.MAV_MODE_AUTO_ARMED],
        CUSTOM_SUBMODE_LANDING_DESCENT:[m.MAV_MODE_GUIDED_ARMED, m.MAV_MODE_AUTO_ARMED],
    }

    ALLOWED_STATES = {
        CUSTOM_SUBMODE_UNINIT:[m.MAV_STATE_UNINIT, m.MAV_STATE_POWEROFF, m.MAV_STATE_CRITICAL],
        CUSTOM_SUBMODE_BOOT:[m.MAV_STATE_BOOT, m.MAV_STATE_CALIBRATING, m.MAV_STATE_STANDBY, m.MAV_STATE_CRITICAL],
        CUSTOM_SUBMODE_SHUTDOWN:[m.MAV_STATE_POWEROFF, m.MAV_STATE_BOOT, m.MAV_STATE_CRITICAL],
        CUSTOM_SUBMODE_GROUND_DISARMED:[m.MAV_STATE_STANDBY, m.MAV_STATE_CRITICAL],
        CUSTOM_SUBMODE_GROUND_ARMED:[m.MAV_STATE_ACTIVE, m.MAV_STATE_CRITICAL, m.MAV_STATE_EMERGENCY],
        CUSTOM_SUBMODE_TAKEOFF_ASCENT:[m.MAV_STATE_ACTIVE, m.MAV_STATE_CRITICAL, m.MAV_STATE_EMERGENCY],
        CUSTOM_SUBMODE_TAKEOFF_HOVER:[m.MAV_STATE_ACTIVE, m.MAV_STATE_CRITICAL, m.MAV_STATE_EMERGENCY],
        CUSTOM_SUBMODE_TAKEOFF_TRANSIT:[m.MAV_STATE_ACTIVE, m.MAV_STATE_CRITICAL, m.MAV_STATE_EMERGENCY],
        CUSTOM_SUBMODE_FLIGHT_NORMAL:[m.MAV_STATE_ACTIVE, m.MAV_STATE_CRITICAL, m.MAV_STATE_EMERGENCY],
        CUSTOM_SUBMODE_FLIGHT_TERRAIN_AVOIDANCE:[m.MAV_STATE_ACTIVE, m.MAV_STATE_CRITICAL, m.MAV_STATE_EMERGENCY],
        CUSTOM_SUBMODE_LANDING_TRANSIT:[m.MAV_STATE_ACTIVE, m.MAV_STATE_CRITICAL, m.MAV_STATE_EMERGENCY],
        CUSTOM_SUBMODE_LANDING_HOVER:[m.MAV_STATE_ACTIVE, m.MAV_STATE_CRITICAL, m.MAV_STATE_EMERGENCY],
        CUSTOM_SUBMODE_LANDING_DESCENT:[m.MAV_STATE_ACTIVE, m.MAV_STATE_CRITICAL, m.MAV_STATE_EMERGENCY],
    }

    # Index 0 is default step up, index 1 is default step down (if applicable)
    ALLOWED_SUBMODE_CHANGES = {
        CUSTOM_SUBMODE_UNINIT:[CUSTOM_SUBMODE_BOOT],
        CUSTOM_SUBMODE_BOOT:[CUSTOM_SUBMODE_GROUND_DISARMED, CUSTOM_SUBMODE_SHUTDOWN],
        CUSTOM_SUBMODE_SHUTDOWN:[CUSTOM_SUBMODE_UNINIT, CUSTOM_SUBMODE_BOOT],
        CUSTOM_SUBMODE_GROUND_DISARMED:[CUSTOM_SUBMODE_GROUND_ARMED, CUSTOM_SUBMODE_SHUTDOWN],
        CUSTOM_SUBMODE_GROUND_ARMED:[CUSTOM_SUBMODE_TAKEOFF_ASCENT, CUSTOM_SUBMODE_GROUND_DISARMED],
        CUSTOM_SUBMODE_TAKEOFF_ASCENT:[CUSTOM_SUBMODE_TAKEOFF_HOVER, CUSTOM_SUBMODE_LANDING_DESCENT],
        CUSTOM_SUBMODE_TAKEOFF_HOVER:[CUSTOM_SUBMODE_TAKEOFF_TRANSIT, CUSTOM_SUBMODE_LANDING_HOVER],
        CUSTOM_SUBMODE_TAKEOFF_TRANSIT:[CUSTOM_SUBMODE_FLIGHT_NORMAL, CUSTOM_SUBMODE_LANDING_TRANSIT],
        CUSTOM_SUBMODE_FLIGHT_NORMAL:[CUSTOM_SUBMODE_LANDING_TRANSIT, CUSTOM_SUBMODE_FLIGHT_TERRAIN_AVOIDANCE, CUSTOM_SUBMODE_FLIGHT_MANUAL],
        CUSTOM_SUBMODE_FLIGHT_MANUAL:[CUSTOM_SUBMODE_FLIGHT_NORMAL],
        CUSTOM_SUBMODE_FLIGHT_TERRAIN_AVOIDANCE:[CUSTOM_SUBMODE_FLIGHT_NORMAL, CUSTOM_SUBMODE_FLIGHT_MANUAL],
        CUSTOM_SUBMODE_LANDING_TRANSIT:[CUSTOM_SUBMODE_LANDING_HOVER, CUSTOM_SUBMODE_TAKEOFF_TRANSIT],
        CUSTOM_SUBMODE_LANDING_HOVER:[CUSTOM_SUBMODE_LANDING_DESCENT, CUSTOM_SUBMODE_TAKEOFF_HOVER],
        CUSTOM_SUBMODE_LANDING_DESCENT:[CUSTOM_SUBMODE_GROUND_ARMED, CUSTOM_SUBMODE_TAKEOFF_ASCENT],
    }

    def __init__(self, state: int, mode: int, custom_mode: int, custom_submode: int, boot: asyncio.Event) -> None:
        state = int(state)
        mode = int(mode)
        custom_mode = int(custom_mode)
        custom_submode = int(custom_submode)

        assert state in GlobalState.MAV_STATES_NOMINAL or state in GlobalState.MAV_STATES_ABNORMAL, 'GlobalState input \'state\' must be MAV_STATE int'
        assert mode in GlobalState.MAV_MODES, 'GlobalState input \'mode\' must be MAV_MODE int'
        assert custom_mode in GlobalState.CUSTOM_MODES, 'GlobalState input \'custom_mode\' must be GlobalState.CUSTOM_MODE int'
        assert custom_submode in GlobalState.CUSTOM_SUBMODES, 'GlobalState input \'custom_submode\' must be GlobalState.CUSTOM_SUBMODE int'
        
        self.state = state
        self.mode = mode
        self.custom_mode = custom_mode
        self.custom_submode = custom_submode
        logging.warning(f'Submode set to: {GlobalState.CUSTOM_SUBMODE_NAMES[self.custom_submode]}')

        self.boot = boot

    def set_mode(self, mode: int, custom_mode: int, custom_submode: int) -> bool:
        try:
            mode = int(mode)
            custom_mode = int(custom_mode)
            custom_submode = int(custom_submode)

            if mode == m.MAV_MODE_MANUAL_ARMED and self.custom_mode == GlobalState.CUSTOM_MODE_FLIGHT:
                self.custom_submode = GlobalState.CUSTOM_SUBMODE_FLIGHT_MANUAL
                logging.warning(f'Submode set to: {GlobalState.CUSTOM_SUBMODE_NAMES[self.custom_submode]}')
                self.mode = mode
                self.state = m.MAV_STATE_ACTIVE
                return True

            if self.custom_submode == GlobalState.CUSTOM_SUBMODE_UNINIT and custom_submode == GlobalState.CUSTOM_SUBMODE_BOOT:
                self.boot.set()

            if custom_submode != self.custom_submode:
                assert custom_submode in GlobalState.ALLOWED_SUBMODE_CHANGES[self.custom_submode]
                self.custom_submode = custom_submode
                logging.warning(f'Submode set to: {GlobalState.CUSTOM_SUBMODE_NAMES[self.custom_submode]}')

                if self.state not in GlobalState.MAV_STATES_ABNORMAL:
                    self.state = GlobalState.ALLOWED_STATES[self.custom_submode][0]

            if mode != self.mode:
                if mode in GlobalState.ALLOWED_MODES[self.custom_submode]:
                    self.mode = mode
                else:
                    self.mode = GlobalState.ALLOWED_MODES[self.custom_submode][0]

            if custom_mode != self.custom_mode:
                if custom_mode in GlobalState.ALLOWED_CUSTOM_MODES[self.custom_submode]:
                    self.custom_mode = custom_mode
                else:
                    self.custom_mode = GlobalState.ALLOWED_CUSTOM_MODES[self.custom_submode][0]

            return True
        except AssertionError:
            return False

    def inc_mode(self) -> None:
        self.custom_submode = GlobalState.ALLOWED_SUBMODE_CHANGES[self.custom_submode][0]
        logging.warning(f'Submode set to: {GlobalState.CUSTOM_SUBMODE_NAMES[self.custom_submode]}')
        self.mode = GlobalState.ALLOWED_MODES[self.custom_submode][0]
        self.custom_mode = GlobalState.ALLOWED_CUSTOM_MODES[self.custom_submode][0]

        if self.state not in GlobalState.MAV_STATES_ABNORMAL:
            self.state = GlobalState.ALLOWED_STATES[self.custom_submode][0]

    def dec_mode(self) -> None:
        if self.custom_submode not in [GlobalState.CUSTOM_SUBMODE_FLIGHT_MANUAL, GlobalState.CUSTOM_SUBMODE_FLIGHT_TERRAIN_AVOIDANCE, GlobalState.CUSTOM_SUBMODE_UNINIT]:
            self.custom_submode = GlobalState.ALLOWED_SUBMODE_CHANGES[self.custom_submode][1]
            logging.warning(f'Submode set to: {GlobalState.CUSTOM_SUBMODE_NAMES[self.custom_submode]}')
            self.mode = GlobalState.ALLOWED_MODES[self.custom_submode][0]
            self.custom_mode = GlobalState.ALLOWED_CUSTOM_MODES[self.custom_submode][0]

            if self.state not in GlobalState.MAV_STATES_ABNORMAL:
                self.state = GlobalState.ALLOWED_STATES[self.custom_submode][0]
        else:
            logging.debug('Invalid submode for GlobalState.dec_mode(), running GlobalState.inc_mode() instead')
            self.inc_mode()

    def set_state(self, state: int) -> None:
        if (state in GlobalState.MAV_STATES_NOMINAL or state in GlobalState.MAV_STATES_ABNORMAL) and state in GlobalState.ALLOWED_STATES[self.custom_submode]:
            self.state = state
        else:
            logging.debug('Invalid state passed into GlobalState.set_state()')
