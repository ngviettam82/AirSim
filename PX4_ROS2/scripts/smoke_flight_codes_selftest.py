"""Two-direction check of the real functions in smoke_flight.py, no ROS needed.

Loads the module source, pulls out the two methods and the constants, and binds them to
a stub. This exercises the shipped code, not a copy of it.
"""
import ast, io, types

SRC = "/mnt/c/code/PX4_ROS2/src/uav_bringup/test/smoke_flight.py"
tree = ast.parse(io.open(SRC, encoding="utf-8").read())

wanted = {"SAFETY_EXPLAINED_CODES", "CAMERA_EXPLAINED_REASONS"}
ns = {}
funcs = {}
for node in tree.body:
    if isinstance(node, ast.Assign) and any(
            getattr(t, "id", None) in wanted for t in node.targets):
        exec(compile(ast.Module([node], []), SRC, "exec"), ns)
    if isinstance(node, ast.ClassDef):
        for sub in node.body:
            if isinstance(sub, ast.FunctionDef) and sub.name in (
                    "safetyUnexplainedCodes", "cameraReasonsOutsideTheExplanation"):
                exec(compile(ast.Module([sub], []), SRC, "exec"), ns)
                funcs[sub.name] = ns[sub.name]

assert set(funcs) == {"safetyUnexplainedCodes", "cameraReasonsOutsideTheExplanation"}, funcs


class Stub:
    pass


def run(codes, reasons):
    s = Stub()
    s.safety_codes = list(codes)
    s.camera_degraded_reasons = set(reasons)
    s.cameraReasonsOutsideTheExplanation = types.MethodType(
        funcs["cameraReasonsOutsideTheExplanation"], s)
    return funcs["safetyUnexplainedCodes"](s)


CASES = [
    ("the traced cause alone is explained",
     ["OFFBOARD_UNHEALTHY", "CAMERA_STREAM_UNHEALTHY"], ["flat image, may be legitimate"], []),
    ("a dead stream is NOT explained",
     ["CAMERA_STREAM_UNHEALTHY"], ["no frames"], ["CAMERA_STREAM_UNHEALTHY"]),
    ("a gap is NOT explained",
     ["CAMERA_STREAM_UNHEALTHY"], ["frames lost: longest gap 1.4 s"],
     ["CAMERA_STREAM_UNHEALTHY"]),
    ("flat PLUS something else is NOT explained",
     ["CAMERA_STREAM_UNHEALTHY"], ["flat image, may be legitimate", "no frames"],
     ["CAMERA_STREAM_UNHEALTHY"]),
    ("no detail at all is NOT explained (O3: not measured is not OK)",
     ["CAMERA_STREAM_UNHEALTHY"], [], ["CAMERA_STREAM_UNHEALTHY"]),
    ("an unrelated new code still blocks",
     ["SOMETHING_NEW"], ["flat image, may be legitimate"], ["SOMETHING_NEW"]),
    ("a clean flight has nothing to explain",
     ["OFFBOARD_UNHEALTHY"], [], []),
]

bad = 0
for name, codes, reasons, expect in CASES:
    got = run(codes, reasons)
    ok = got == expect
    print("  %-4s %-58s got=%s" % ("ok" if ok else "FAIL", name, got))
    if not ok:
        bad += 1
        print("        expected %s" % (expect,))

print()
print("CAMERA GUARD: %d ok / %d failed" % (len(CASES) - bad, bad))
raise SystemExit(1 if bad else 0)
