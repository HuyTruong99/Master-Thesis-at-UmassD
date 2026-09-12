import socket
import threading
import re
import time
import csv
import json
import hashlib
import shutil

from pathlib import Path
from datetime import datetime


# ============================================================
# FILES
# ============================================================

BASE_DIR = Path(__file__).resolve().parent
POLICY_FILE = BASE_DIR / "safety_policy.json"
CSV_FILE = BASE_DIR / "robot_position.csv"


# ============================================================
# ROBOT NETWORK
# ============================================================

ROBOT_IP = "192.168.1.6"
PORT = 2000


# ============================================================
# LOAD POLICY
# ============================================================

def load_policy():
    if not POLICY_FILE.exists():
        raise FileNotFoundError(f"Cannot find: {POLICY_FILE}")

    with open(POLICY_FILE, "r", encoding="utf-8") as f:
        return json.load(f)


POLICY = load_policy()

POLICY_NAME = POLICY.get("policy_name", "unnamed_policy")
MODE = POLICY.get("mode", "monitor").lower()

if MODE not in ("measure", "monitor", "enforce"):
    raise ValueError("mode must be measure, monitor, or enforce")

MAX_MOVE_MM = float(POLICY["max_move_mm"])
WARNING_MARGIN_MM = float(POLICY["warning_margin_mm"])

WX_MIN = POLICY["workspace"]["x_min"]
WX_MAX = POLICY["workspace"]["x_max"]
WY_MIN = POLICY["workspace"]["y_min"]
WY_MAX = POLICY["workspace"]["y_max"]
WZ_MIN = POLICY["workspace"]["z_min"]
WZ_MAX = POLICY["workspace"]["z_max"]

RX_MIN = POLICY["runtime_stop"]["x_min"]
RX_MAX = POLICY["runtime_stop"]["x_max"]
RY_MIN = POLICY["runtime_stop"]["y_min"]
RY_MAX = POLICY["runtime_stop"]["y_max"]
RZ_MIN = POLICY["runtime_stop"]["z_min"]
RZ_MAX = POLICY["runtime_stop"]["z_max"]

AUTO_SUPERVISOR_STOP = (
    MODE == "enforce"
    and bool(POLICY.get("auto_supervisor_stop", False))
)


# ============================================================
# POLICY VALIDATION
# ============================================================

def validate_axis_policy(axis, workspace_min, workspace_max, stop_min, stop_max):
    values = (workspace_min, workspace_max, stop_min, stop_max)

    if any(v is None for v in values):
        return

    if not (workspace_min < stop_min < stop_max < workspace_max):
        raise ValueError(
            f"{axis}: expected workspace_min < stop_min < stop_max < workspace_max"
        )

    if stop_min + WARNING_MARGIN_MM >= stop_max - WARNING_MARGIN_MM:
        raise ValueError(f"{axis}: warning margin is too large")


def validate_policy():
    if MAX_MOVE_MM <= 0:
        raise ValueError("max_move_mm must be > 0")

    if WARNING_MARGIN_MM < 0:
        raise ValueError("warning_margin_mm cannot be negative")

    validate_axis_policy("X", WX_MIN, WX_MAX, RX_MIN, RX_MAX)
    validate_axis_policy("Y", WY_MIN, WY_MAX, RY_MIN, RY_MAX)
    validate_axis_policy("Z", WZ_MIN, WZ_MAX, RZ_MIN, RZ_MAX)


validate_policy()


# ============================================================
# POLICY HASH / SESSION
# ============================================================

POLICY_TEXT = json.dumps(POLICY, sort_keys=True)
POLICY_HASH = hashlib.sha256(POLICY_TEXT.encode("utf-8")).hexdigest()
SESSION_ID = datetime.now().strftime("%Y%m%d_%H%M%S")


# ============================================================
# SHARED STATE
# ============================================================

robot_socket = None
shutdown_event = threading.Event()

send_lock = threading.Lock()
csv_lock = threading.Lock()

trial_counter = 0

current_trial_id = ""
current_user_input = ""
current_command = ""
current_command_safety = "NOT_APPLICABLE"

current_axis = ""
requested_distance = None

movement_active = False
movement_start_time = None
sample_index = 0

start_x = None
start_y = None
start_z = None

last_x = None
last_y = None
last_z = None
last_u = None
last_v = None
last_w = None

supervisor_hold_active = False

warning_reported = False
stop_reported = False
workspace_reported = False

stop_request_time = None
stop_request_x = None
stop_request_y = None
stop_request_z = None

awaiting_final_stop_position = False


# ============================================================
# CSV
# ============================================================

CSV_HEADER = [
    "timestamp",
    "session_id",
    "policy_name",
    "policy_sha256",
    "policy_mode",

    "trial_id",
    "sample_index",
    "elapsed_ms",

    "event",
    "user_input",
    "command",
    "axis",
    "requested_distance_mm",

    "predicted_x",
    "predicted_y",
    "predicted_z",

    "x",
    "y",
    "z",
    "u",
    "v",
    "w",

    "actual_axis_displacement_mm",

    "status",
    "validation",

    "command_safety",
    "virtual_wall_state",
    "runtime_intervention_state",

    "reason",

    "stop_response_ms",
    "post_stop_travel_mm",
    "threshold_overshoot_mm",

    "raw_message"
]


def now():
    return datetime.now().isoformat(timespec="milliseconds")


def initialize_csv():
    if CSV_FILE.exists():
        with open(CSV_FILE, "r", newline="", encoding="utf-8") as f:
            old_header = next(csv.reader(f), [])

        if old_header != CSV_HEADER:
            backup = BASE_DIR / (
                "robot_position_backup_"
                + datetime.now().strftime("%Y%m%d_%H%M%S")
                + ".csv"
            )
            shutil.move(str(CSV_FILE), str(backup))
            print("Old CSV backed up:", backup)

    if not CSV_FILE.exists():
        with open(CSV_FILE, "w", newline="", encoding="utf-8") as f:
            csv.writer(f).writerow(CSV_HEADER)


def log_csv(
    event,
    trial_id="",
    sample="",
    elapsed_ms="",
    user_input="",
    command="",
    axis="",
    distance="",
    predicted_x="",
    predicted_y="",
    predicted_z="",
    x="",
    y="",
    z="",
    u="",
    v="",
    w="",
    displacement="",
    status="",
    validation="",
    command_safety="NOT_APPLICABLE",
    virtual_wall_state="",
    runtime_intervention_state="NONE",
    reason="",
    stop_response_ms="",
    post_stop_travel_mm="",
    threshold_overshoot_mm="",
    raw_message=""
):
    row = [
        now(),
        SESSION_ID,
        POLICY_NAME,
        POLICY_HASH,
        MODE,

        trial_id,
        sample,
        elapsed_ms,

        event,
        user_input,
        command,
        axis,
        distance,

        predicted_x,
        predicted_y,
        predicted_z,

        x,
        y,
        z,
        u,
        v,
        w,

        displacement,

        status,
        validation,

        command_safety,
        virtual_wall_state,
        runtime_intervention_state,

        reason,

        stop_response_ms,
        post_stop_travel_mm,
        threshold_overshoot_mm,

        raw_message
    ]

    try:
        with csv_lock:
            with open(CSV_FILE, "a", newline="", encoding="utf-8") as f:
                csv.writer(f).writerow(row)

    except PermissionError:
        print("CSV WRITE ERROR: close robot_position.csv in Excel.")


# ============================================================
# NETWORK
# ============================================================

def send_line(command):
    with send_lock:
        robot_socket.sendall((command + "\r\n").encode("utf-8"))


# ============================================================
# POSITION PARSER
# ============================================================

def parse_position(line):
    parts = [p.strip() for p in line.split(",")]

    if len(parts) != 7 or parts[0].upper() != "POS":
        return None

    try:
        return {
            "x": float(parts[1]),
            "y": float(parts[2]),
            "z": float(parts[3]),
            "u": float(parts[4]),
            "v": float(parts[5]),
            "w": float(parts[6]),
        }
    except ValueError:
        return None


# ============================================================
# VIRTUAL-WALL CLASSIFICATION
#
# This is YOUR experimental policy state.
# It is NOT an Epson manufacturer safety status.
# ============================================================

def axis_virtual_state(value, workspace_min, workspace_max, stop_min, stop_max, axis):
    reasons = []

    if workspace_min is not None and value < workspace_min:
        return "OUTSIDE_WORKSPACE", [f"{axis}_BELOW_WORKSPACE_MIN:{value:.3f}"]

    if workspace_max is not None and value > workspace_max:
        return "OUTSIDE_WORKSPACE", [f"{axis}_ABOVE_WORKSPACE_MAX:{value:.3f}"]

    if stop_min is not None and value <= stop_min:
        return "STOP_ZONE", [f"{axis}_LOW_STOP_ZONE:{value:.3f}"]

    if stop_max is not None and value >= stop_max:
        return "STOP_ZONE", [f"{axis}_HIGH_STOP_ZONE:{value:.3f}"]

    if stop_min is not None and value <= stop_min + WARNING_MARGIN_MM:
        reasons.append(f"{axis}_LOW_WARNING:{value:.3f}")

    if stop_max is not None and value >= stop_max - WARNING_MARGIN_MM:
        reasons.append(f"{axis}_HIGH_WARNING:{value:.3f}")

    if reasons:
        return "WARNING", reasons

    return "SAFE", []


def classify_virtual_wall_state(x, y, z):
    results = [
        axis_virtual_state(x, WX_MIN, WX_MAX, RX_MIN, RX_MAX, "X"),
        axis_virtual_state(y, WY_MIN, WY_MAX, RY_MIN, RY_MAX, "Y"),
        axis_virtual_state(z, WZ_MIN, WZ_MAX, RZ_MIN, RZ_MAX, "Z"),
    ]

    priority = {
        "SAFE": 0,
        "WARNING": 1,
        "STOP_ZONE": 2,
        "OUTSIDE_WORKSPACE": 3,
    }

    highest = max(results, key=lambda item: priority[item[0]])[0]
    reasons = []

    for state, state_reasons in results:
        if priority[state] == priority[highest]:
            reasons.extend(state_reasons)

    return highest, ";".join(reasons)


# ============================================================
# DIRECTION-AWARE RUNTIME INTERVENTION
#
# This determines whether the CURRENT motion is moving toward
# a virtual wall and should warn / stop.
# ============================================================

def runtime_intervention_state(x, y, z):
    if MODE == "measure" or not movement_active:
        return "NONE", ""

    virtual_state, virtual_reason = classify_virtual_wall_state(x, y, z)

    if virtual_state == "OUTSIDE_WORKSPACE":
        return "WORKSPACE_VIOLATION", virtual_reason

    if requested_distance is None:
        return "NONE", ""

    value = None
    stop_min = None
    stop_max = None
    axis = current_axis

    if axis == "X":
        value, stop_min, stop_max = x, RX_MIN, RX_MAX
    elif axis == "Y":
        value, stop_min, stop_max = y, RY_MIN, RY_MAX
    elif axis == "Z":
        value, stop_min, stop_max = z, RZ_MIN, RZ_MAX
    else:
        return "NONE", ""

    if requested_distance < 0 and stop_min is not None:
        if value <= stop_min:
            return "STOP_THRESHOLD", f"{axis}_MIN_STOP:{value:.3f}"
        if value <= stop_min + WARNING_MARGIN_MM:
            return "WARNING", f"{axis}_MIN_WARNING:{value:.3f}"

    if requested_distance > 0 and stop_max is not None:
        if value >= stop_max:
            return "STOP_THRESHOLD", f"{axis}_MAX_STOP:{value:.3f}"
        if value >= stop_max - WARNING_MARGIN_MM:
            return "WARNING", f"{axis}_MAX_WARNING:{value:.3f}"

    return "NONE", ""


# ============================================================
# DISPLACEMENT / TARGET
# ============================================================

def axis_displacement():
    if current_axis == "X" and start_x is not None and last_x is not None:
        return last_x - start_x

    if current_axis == "Y" and start_y is not None and last_y is not None:
        return last_y - start_y

    if current_axis == "Z" and start_z is not None and last_z is not None:
        return last_z - start_z

    return ""


def predict_target(axis, distance):
    if last_x is None or last_y is None or last_z is None:
        return None

    x, y, z = last_x, last_y, last_z

    if axis == "X":
        x += distance
    elif axis == "Y":
        y += distance
    elif axis == "Z":
        z += distance

    return x, y, z


# ============================================================
# PRE-ACTUATION COMMAND VALIDATION
# ============================================================

def validate_move(axis, distance):
    if supervisor_hold_active:
        return False, "UNSAFE", "SUPERVISOR_HOLD_ACTIVE", None, None

    if axis not in ("X", "Y", "Z"):
        return False, "UNSAFE", "INVALID_AXIS", None, None

    if distance == 0:
        return False, "UNSAFE", "ZERO_DISTANCE", None, None

    if abs(distance) > MAX_MOVE_MM:
        return (
            False,
            "UNSAFE",
            f"DISTANCE_LIMIT:{distance};MAX={MAX_MOVE_MM}",
            None,
            None,
        )

    target = predict_target(axis, distance)

    if target is None:
        return False, "UNSAFE", "CURRENT_POSITION_UNKNOWN", None, None

    target_state, target_reason = classify_virtual_wall_state(*target)

    if target_state == "OUTSIDE_WORKSPACE":
        return (
            False,
            "UNSAFE",
            "PREDICTED_WORKSPACE_VIOLATION:" + target_reason,
            target,
            target_state,
        )

    # In ENFORCE mode, do not intentionally command a target
    # that lies inside the virtual stop zone.
    if MODE == "enforce" and target_state == "STOP_ZONE":
        return (
            False,
            "UNSAFE",
            "PREDICTED_STOP_ZONE:" + target_reason,
            target,
            target_state,
        )

    if target_state == "STOP_ZONE":
        return True, "UNSAFE_TEST_ALLOWED", "MONITOR_MODE_STOP_ZONE_TEST", target, target_state

    if target_state == "WARNING":
        return True, "CAUTION", "PREDICTED_WARNING_ZONE", target, target_state

    return True, "SAFE", "", target, target_state


# ============================================================
# SOFTWARE SUPERVISOR HOLD
# ============================================================

def request_supervisor_hold(reason):
    global supervisor_hold_active
    global stop_request_time
    global stop_request_x, stop_request_y, stop_request_z

    if supervisor_hold_active:
        return

    supervisor_hold_active = True

    stop_request_time = time.monotonic()
    stop_request_x = last_x
    stop_request_y = last_y
    stop_request_z = last_z

    current_virtual_state, _ = classify_virtual_wall_state(
        last_x, last_y, last_z
    )

    print()
    print("================================")
    print("SOFTWARE SUPERVISOR HOLD REQUESTED")
    print("================================")
    print(reason)

    log_csv(
        event="SUPERVISOR_HOLD_REQUESTED",
        trial_id=current_trial_id,
        user_input=current_user_input,
        command=current_command,
        axis=current_axis,
        distance=requested_distance,
        x=last_x,
        y=last_y,
        z=last_z,
        status="STOP_REQUESTED",
        validation="FAIL",
        command_safety=current_command_safety,
        virtual_wall_state=current_virtual_state,
        runtime_intervention_state="STOP_REQUESTED",
        reason=reason,
    )

    send_line("SUPERVISOR_STOP")


# ============================================================
# STOP METRICS
# ============================================================

def post_stop_travel():
    if current_axis == "X" and stop_request_x is not None and last_x is not None:
        return abs(last_x - stop_request_x)

    if current_axis == "Y" and stop_request_y is not None and last_y is not None:
        return abs(last_y - stop_request_y)

    if current_axis == "Z" and stop_request_z is not None and last_z is not None:
        return abs(last_z - stop_request_z)

    return ""


def threshold_overshoot():
    if requested_distance is None:
        return ""

    if current_axis == "X":
        if requested_distance < 0 and RX_MIN is not None:
            return max(0, RX_MIN - last_x)
        if requested_distance > 0 and RX_MAX is not None:
            return max(0, last_x - RX_MAX)

    if current_axis == "Y":
        if requested_distance < 0 and RY_MIN is not None:
            return max(0, RY_MIN - last_y)
        if requested_distance > 0 and RY_MAX is not None:
            return max(0, last_y - RY_MAX)

    if current_axis == "Z":
        if requested_distance < 0 and RZ_MIN is not None:
            return max(0, RZ_MIN - last_z)
        if requested_distance > 0 and RZ_MAX is not None:
            return max(0, last_z - RZ_MAX)

    return ""


# ============================================================
# RC+ MESSAGE PROCESSING
# ============================================================

def process_robot_message(line):
    global movement_active, movement_start_time, sample_index
    global start_x, start_y, start_z
    global last_x, last_y, last_z, last_u, last_v, last_w
    global warning_reported, stop_reported, workspace_reported
    global supervisor_hold_active, awaiting_final_stop_position

    pos = parse_position(line)

    # --------------------------------------------------------
    # POSITION
    # --------------------------------------------------------
    if pos is not None:
        last_x = pos["x"]
        last_y = pos["y"]
        last_z = pos["z"]
        last_u = pos["u"]
        last_v = pos["v"]
        last_w = pos["w"]

        virtual_state, virtual_reason = classify_virtual_wall_state(
            last_x, last_y, last_z
        )

        intervention_state, intervention_reason = runtime_intervention_state(
            last_x, last_y, last_z
        )

        if awaiting_final_stop_position:
            awaiting_final_stop_position = False

            travel = post_stop_travel()
            overshoot = threshold_overshoot()

            print()
            print("FINAL STOPPED POSITION")
            print(f"X={last_x:.3f} Y={last_y:.3f} Z={last_z:.3f}")
            print("Post-stop travel:", travel, "mm")
            print("Threshold overshoot:", overshoot, "mm")

            log_csv(
                event="STOPPED_FINAL_POSITION",
                trial_id=current_trial_id,
                user_input=current_user_input,
                command=current_command,
                axis=current_axis,
                distance=requested_distance,
                x=last_x,
                y=last_y,
                z=last_z,
                u=last_u,
                v=last_v,
                w=last_w,
                displacement=axis_displacement(),
                status="STOPPED",
                validation="FAIL",
                command_safety=current_command_safety,
                virtual_wall_state=virtual_state,
                runtime_intervention_state="STOPPED",
                post_stop_travel_mm=travel,
                threshold_overshoot_mm=overshoot,
                raw_message=line,
            )

        sample = ""
        elapsed = ""

        if movement_active:
            sample_index += 1
            sample = sample_index
            elapsed = round(
                (time.monotonic() - movement_start_time) * 1000,
                2
            )

        reason_parts = []
        if virtual_reason:
            reason_parts.append("VIRTUAL:" + virtual_reason)
        if intervention_reason:
            reason_parts.append("RUNTIME:" + intervention_reason)

        log_csv(
            event="POSITION",
            trial_id=current_trial_id,
            sample=sample,
            elapsed_ms=elapsed,
            user_input=current_user_input,
            command=current_command,
            axis=current_axis,
            distance=requested_distance,
            x=last_x,
            y=last_y,
            z=last_z,
            u=last_u,
            v=last_v,
            w=last_w,
            displacement=axis_displacement(),
            status="MOVING" if movement_active else "IDLE",
            validation="PASS",
            command_safety=current_command_safety,
            virtual_wall_state=virtual_state,
            runtime_intervention_state=intervention_state,
            reason=";".join(reason_parts),
            raw_message=line,
        )

        if (
            movement_active
            and intervention_state == "WARNING"
            and not warning_reported
        ):
            warning_reported = True
            print()
            print("RUNTIME WARNING:", intervention_reason)

            log_csv(
                event="RUNTIME_WARNING",
                trial_id=current_trial_id,
                sample=sample,
                elapsed_ms=elapsed,
                x=last_x,
                y=last_y,
                z=last_z,
                status="WARNING",
                validation="PASS",
                command_safety=current_command_safety,
                virtual_wall_state=virtual_state,
                runtime_intervention_state="WARNING",
                reason=intervention_reason,
            )

        if (
            movement_active
            and intervention_state == "STOP_THRESHOLD"
            and not stop_reported
        ):
            stop_reported = True
            print()
            print("RUNTIME STOP THRESHOLD:", intervention_reason)

            log_csv(
                event="RUNTIME_STOP_THRESHOLD",
                trial_id=current_trial_id,
                sample=sample,
                elapsed_ms=elapsed,
                user_input=current_user_input,
                command=current_command,
                axis=current_axis,
                distance=requested_distance,
                x=last_x,
                y=last_y,
                z=last_z,
                status="STOP_THRESHOLD",
                validation="FAIL",
                command_safety=current_command_safety,
                virtual_wall_state=virtual_state,
                runtime_intervention_state="STOP_THRESHOLD",
                reason=intervention_reason,
            )

            if AUTO_SUPERVISOR_STOP:
                request_supervisor_hold(intervention_reason)
            else:
                print("MONITOR MODE: threshold logged only; robot continues.")

        if (
            movement_active
            and intervention_state == "WORKSPACE_VIOLATION"
            and not workspace_reported
        ):
            workspace_reported = True
            print()
            print("WORKSPACE VIOLATION:", intervention_reason)

            log_csv(
                event="WORKSPACE_VIOLATION",
                trial_id=current_trial_id,
                sample=sample,
                elapsed_ms=elapsed,
                x=last_x,
                y=last_y,
                z=last_z,
                status="CRITICAL",
                validation="FAIL",
                command_safety=current_command_safety,
                virtual_wall_state=virtual_state,
                runtime_intervention_state="WORKSPACE_VIOLATION",
                reason=intervention_reason,
            )

            if AUTO_SUPERVISOR_STOP:
                request_supervisor_hold(intervention_reason)

        return

    # --------------------------------------------------------
    # START
    # --------------------------------------------------------
    if line.upper().startswith("START"):
        movement_active = True
        movement_start_time = time.monotonic()

        sample_index = 0
        warning_reported = False
        stop_reported = False
        workspace_reported = False

        start_x = last_x
        start_y = last_y
        start_z = last_z

        virtual_state, _ = classify_virtual_wall_state(
            last_x, last_y, last_z
        )

        log_csv(
            event="MOVE_START",
            trial_id=current_trial_id,
            user_input=current_user_input,
            command=current_command,
            axis=current_axis,
            distance=requested_distance,
            x=start_x,
            y=start_y,
            z=start_z,
            status="MOVING",
            validation="PASS",
            command_safety=current_command_safety,
            virtual_wall_state=virtual_state,
            runtime_intervention_state="NONE",
            raw_message=line,
        )
        return

    # --------------------------------------------------------
    # DONE
    # --------------------------------------------------------
    if line.upper().startswith("DONE"):
        movement_active = False

        virtual_state, virtual_reason = classify_virtual_wall_state(
            last_x, last_y, last_z
        )

        log_csv(
            event="MOVE_DONE",
            trial_id=current_trial_id,
            user_input=current_user_input,
            command=current_command,
            axis=current_axis,
            distance=requested_distance,
            x=last_x,
            y=last_y,
            z=last_z,
            displacement=axis_displacement(),
            status="DONE",
            validation="PASS",
            command_safety=current_command_safety,
            virtual_wall_state=virtual_state,
            runtime_intervention_state="NONE",
            reason=virtual_reason,
            raw_message=line,
        )
        return

    # --------------------------------------------------------
    # ABORTED
    # --------------------------------------------------------
    if line.upper().startswith("MOTION_ABORTED"):
        movement_active = False
        awaiting_final_stop_position = True

        response_ms = ""

        if stop_request_time is not None:
            response_ms = round(
                (time.monotonic() - stop_request_time) * 1000,
                2
            )

        virtual_state, _ = classify_virtual_wall_state(
            last_x, last_y, last_z
        )

        print()
        print("MOTION ABORTED")
        print("Abort response:", response_ms, "ms")

        log_csv(
            event="MOTION_ABORTED",
            trial_id=current_trial_id,
            user_input=current_user_input,
            command=current_command,
            axis=current_axis,
            distance=requested_distance,
            x=last_x,
            y=last_y,
            z=last_z,
            displacement=axis_displacement(),
            status="ABORTED",
            validation="FAIL",
            command_safety=current_command_safety,
            virtual_wall_state=virtual_state,
            runtime_intervention_state="ABORTED",
            reason=line,
            stop_response_ms=response_ms,
            raw_message=line,
        )
        return

    # --------------------------------------------------------
    # SUPERVISOR HOLD
    # --------------------------------------------------------
    if (
        line.upper().startswith("SUPERVISOR_HOLD")
        or line.upper().startswith("SAFE_MODE")
    ):
        upper = line.upper()

        if "RESET" in upper:
            supervisor_hold_active = False
            hold_status = "RESET"
        else:
            supervisor_hold_active = True
            hold_status = "ACTIVE"

        virtual_state = ""

        if last_x is not None and last_y is not None and last_z is not None:
            virtual_state, _ = classify_virtual_wall_state(
                last_x, last_y, last_z
            )

        log_csv(
            event="SUPERVISOR_HOLD_EVENT",
            trial_id=current_trial_id,
            status=hold_status,
            validation="PASS",
            command_safety=current_command_safety,
            virtual_wall_state=virtual_state,
            runtime_intervention_state="HOLD",
            raw_message=line,
        )
        return

    # --------------------------------------------------------
    # RC+ REJECT
    # --------------------------------------------------------
    if line.upper().startswith("REJECT"):
        virtual_state = ""

        if last_x is not None and last_y is not None and last_z is not None:
            virtual_state, _ = classify_virtual_wall_state(
                last_x, last_y, last_z
            )

        log_csv(
            event="RC_REJECT",
            trial_id=current_trial_id,
            user_input=current_user_input,
            command=current_command,
            axis=current_axis,
            distance=requested_distance,
            status="BLOCKED",
            validation="FAIL",
            command_safety="UNSAFE",
            virtual_wall_state=virtual_state,
            runtime_intervention_state="NONE",
            reason=line,
            raw_message=line,
        )
        return

    log_csv(
        event="RC_MESSAGE",
        status="INFO",
        validation="PASS",
        command_safety="NOT_APPLICABLE",
        virtual_wall_state="",
        runtime_intervention_state="NONE",
        raw_message=line,
    )


# ============================================================
# RECEIVER
# ============================================================

def receiver():
    buffer = ""

    while not shutdown_event.is_set():
        try:
            data = robot_socket.recv(4096)

            if not data:
                break

            buffer += data.decode("utf-8", errors="ignore")

            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                line = line.strip()

                if line:
                    print(f"\nRC+: {line}")
                    process_robot_message(line)

        except OSError:
            break


# ============================================================
# NATURAL-LANGUAGE PARSER
# ============================================================

def parse_user_command(text):
    text = text.lower().strip()
    text = re.sub(r"-\s+(\d)", r"-\1", text)

    if text in (
        "position",
        "get position",
        "get pos",
        "where am i",
        "where am i now",
    ):
        return "POSITION", None, None

    if text in (
        "stop",
        "stop robot",
        "supervisor stop",
        "safe mode",
    ):
        return "STOP", None, None

    if text in (
        "reset",
        "reset hold",
        "reset supervisor hold",
        "reset safe mode",
        "reset safety",
    ):
        return "RESET", None, None

    numbers = re.findall(r"-?\d+(?:\.\d+)?", text)

    if not numbers:
        return None, None, None

    entered = float(numbers[0])
    magnitude = abs(entered)

    if "right" in text:
        return "MOVE", "X", magnitude

    if "left" in text:
        return "MOVE", "X", -magnitude

    if "forward" in text:
        return "MOVE", "Y", magnitude

    if "backward" in text or "backwards" in text or "back" in text:
        return "MOVE", "Y", -magnitude

    if "up" in text:
        return "MOVE", "Z", magnitude

    if "down" in text:
        return "MOVE", "Z", -magnitude

    if re.search(r"\bx\b", text):
        return "MOVE", "X", entered

    if re.search(r"\by\b", text):
        return "MOVE", "Y", entered

    if re.search(r"\bz\b", text):
        return "MOVE", "Z", entered

    return None, None, None


# ============================================================
# SEND MOVE
# ============================================================

def send_move(axis, distance, user_input):
    global trial_counter
    global current_trial_id, current_user_input, current_command
    global current_command_safety
    global current_axis, requested_distance

    if movement_active:
        print("BLOCKED: robot is already moving.")
        return

    trial_counter += 1
    trial = f"T{trial_counter:04d}"
    command = f"MOVE_{axis},{distance}"

    allow, command_safety, reason, target, target_state = validate_move(
        axis, distance
    )

    current_virtual_state = ""

    if last_x is not None and last_y is not None and last_z is not None:
        current_virtual_state, _ = classify_virtual_wall_state(
            last_x, last_y, last_z
        )

    predicted_x = predicted_y = predicted_z = ""

    if target is not None:
        predicted_x, predicted_y, predicted_z = target

    if not allow:
        print()
        print("==============================")
        print("COMMAND BLOCKED")
        print("==============================")
        print("Command safety:", command_safety)
        print("Current virtual-wall state:", current_virtual_state)
        print("Reason:", reason)

        log_csv(
            event="COMMAND_BLOCKED",
            trial_id=trial,
            user_input=user_input,
            command=command,
            axis=axis,
            distance=distance,
            predicted_x=predicted_x,
            predicted_y=predicted_y,
            predicted_z=predicted_z,
            x=last_x,
            y=last_y,
            z=last_z,
            status="BLOCKED",
            validation="FAIL",
            command_safety=command_safety,
            virtual_wall_state=current_virtual_state,
            runtime_intervention_state="NONE",
            reason=reason,
        )
        return

    current_trial_id = trial
    current_user_input = user_input
    current_command = command
    current_command_safety = command_safety

    current_axis = axis
    requested_distance = distance

    print()
    print("Trial:", trial)
    print("Sending:", command)
    print("Command safety:", command_safety)
    print("Current virtual-wall state:", current_virtual_state)

    if target is not None:
        print(
            f"Predicted target: X={target[0]:.3f} "
            f"Y={target[1]:.3f} Z={target[2]:.3f}"
        )
        print("Predicted virtual-wall state:", target_state)

    log_csv(
        event="COMMAND_SENT",
        trial_id=trial,
        user_input=user_input,
        command=command,
        axis=axis,
        distance=distance,
        predicted_x=predicted_x,
        predicted_y=predicted_y,
        predicted_z=predicted_z,
        x=last_x,
        y=last_y,
        z=last_z,
        status="SENT",
        validation="PASS",
        command_safety=command_safety,
        virtual_wall_state=current_virtual_state,
        runtime_intervention_state="NONE",
        reason=reason,
    )

    send_line(command)


# ============================================================
# DISPLAY POLICY
# ============================================================

def axis_safe_interval(stop_min, stop_max):
    if stop_min is None or stop_max is None:
        return None

    return (
        stop_min + WARNING_MARGIN_MM,
        stop_max - WARNING_MARGIN_MM,
    )


def show_axis(axis, workspace_min, workspace_max, stop_min, stop_max):
    print()
    print(f"{axis} workspace:    {workspace_min} to {workspace_max}")
    print(f"{axis} runtime stop: {stop_min} to {stop_max}")

    safe = axis_safe_interval(stop_min, stop_max)

    if safe is None:
        print(f"{axis} SAFE interval: DISABLED / INCOMPLETE")
    else:
        print(f"{axis} SAFE interval: {safe[0]} to {safe[1]}")


def show_policy():
    print()
    print("==========================================")
    print("XYZ VIRTUAL-WALL RUNTIME SUPERVISOR")
    print("==========================================")
    print("Policy file:", POLICY_FILE)
    print("Policy:", POLICY_NAME)
    print("Mode:", MODE)
    print("Policy SHA256:", POLICY_HASH)

    show_axis("X", WX_MIN, WX_MAX, RX_MIN, RX_MAX)
    show_axis("Y", WY_MIN, WY_MAX, RY_MIN, RY_MAX)
    show_axis("Z", WZ_MIN, WZ_MAX, RZ_MIN, RZ_MAX)

    print()
    print("Warning margin:", WARNING_MARGIN_MM, "mm")
    print("Automatic supervisor stop:", AUTO_SUPERVISOR_STOP)

    if MODE == "monitor":
        print("MONITOR MODE: virtual-wall violations are logged,")
        print("but Python does not automatically abort motion.")

    print()
    print("IMPORTANT:")
    print("virtual_wall_state = YOUR experimental software policy.")
    print("It is NOT an Epson manufacturer safety certification.")
    print()


# ============================================================
# MAIN
# ============================================================

def main():
    global robot_socket

    initialize_csv()
    show_policy()

    robot_socket = socket.socket(
        socket.AF_INET,
        socket.SOCK_STREAM
    )

    try:
        robot_socket.connect((ROBOT_IP, PORT))

    except Exception as error:
        print("Connection failed:", error)
        return

    print("Connected to RC+7.")

    threading.Thread(
        target=receiver,
        daemon=True
    ).start()

    time.sleep(1.0)
    send_line("GET_POSITION")
    time.sleep(0.5)

    print()
    print("Commands:")
    print("  move right 20 mm")
    print("  move left 20 mm")
    print("  move forward 20 mm")
    print("  move backward 20 mm")
    print("  move up 20 mm")
    print("  move down 20 mm")
    print("  get position")
    print("  stop")
    print("  reset")
    print("  quit")

    while True:
        try:
            user_input = input("\nCommand: ").strip()

            if not user_input:
                continue

            if user_input.lower() in ("quit", "exit"):
                break

            command_type, axis, distance = parse_user_command(
                user_input
            )

            if command_type == "POSITION":
                send_line("GET_POSITION")

            elif command_type == "STOP":
                request_supervisor_hold(
                    "MANUAL_OPERATOR_STOP"
                )

            elif command_type == "RESET":
                send_line("RESET_SUPERVISOR_HOLD")

            elif command_type == "MOVE":
                send_move(
                    axis,
                    distance,
                    user_input
                )

            else:
                print("Command not understood.")

                virtual_state = ""

                if (
                    last_x is not None
                    and last_y is not None
                    and last_z is not None
                ):
                    virtual_state, _ = classify_virtual_wall_state(
                        last_x, last_y, last_z
                    )

                log_csv(
                    event="UNKNOWN_USER_COMMAND",
                    user_input=user_input,
                    status="NOT_SENT",
                    validation="FAIL",
                    command_safety="UNSAFE",
                    virtual_wall_state=virtual_state,
                    runtime_intervention_state="NONE",
                    reason="UNKNOWN_OR_AMBIGUOUS_COMMAND",
                )

        except KeyboardInterrupt:
            break

        except Exception as error:
            print("Command error:", error)

    shutdown_event.set()

    try:
        robot_socket.shutdown(socket.SHUT_RDWR)
    except OSError:
        pass

    robot_socket.close()

    log_csv(
        event="PROGRAM_END",
        status="CLOSED",
        validation="PASS",
        command_safety="NOT_APPLICABLE",
        virtual_wall_state="",
        runtime_intervention_state="NONE",
    )

    print()
    print("CSV saved at:")
    print(CSV_FILE)


if __name__ == "__main__":
    main()
