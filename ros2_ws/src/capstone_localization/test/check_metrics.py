"""Exercise the evaluator's CSV arithmetic and its input rejection."""
import csv
import json
import math
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory


def write(path, header, rows):
    with path.open("w") as out:
        writer = csv.writer(out)
        writer.writerow(header)
        writer.writerows(rows)


def main(executable):
    with TemporaryDirectory() as folder:
        here = Path(folder)
        truth, predicted = here/"truth.csv", here/"replay.csv"
        stamps = [i/20 for i in range(61)]
        write(truth, ["stamp","x","y","yaw"], [(t,t,0,0) for t in stamps])
        pose = ["kind","stamp","x","y","yaw"]
        covariance = [f"p{i}{j}" for i in range(3) for j in range(3)]
        diagonal = [1 if i==j else 0 for i in range(3) for j in range(3)]
        names = pose+covariance+["latency_us"]
        rows = [["W",t,t+.1,0,0,*diagonal,1000] for t in stamps]
        write(predicted,names,rows)
        out = json.loads(subprocess.check_output([executable,truth,predicted,"500000000","1500000000"],text=True))
        assert out["samples_window"] == 20
        assert math.isclose(out["ate_rmse_window_m"],.1)
        assert math.isclose(out["nees_mean"],.01)
        assert abs(out["rpe_1s_translation_rmse_m"]) < 1e-12
        assert out["nis_mean"] is None and out["fallback_fraction"] is None
        rows[0][5] = -1
        write(predicted,names,rows)
        bad = subprocess.run([executable,truth,predicted],capture_output=True,text=True)
        assert bad.returncode != 0 and "covariance" in bad.stderr
        pairs = here/"pairs.csv"
        write(pairs,["layout","A_m","B_m"],[("l0",1,3),("l0",1,3),("l1",1,2)])
        out = json.loads(subprocess.check_output([executable,"--paired",pairs],text=True))
        assert out["pairs"]==3 and math.isclose(out["B_minus_A_m"],1.5)
        assert all(math.isclose(x,1.5) for x in out["bootstrap_95_m"])
        estimate, reference = here/"map.txt", here/"ref.txt"
        estimate.write_text("4 1 .05 0 0 0\n100 -1 0 100\n")
        reference.write_text("4 1 .05 0 0 0\n100 100 100 0\n")
        out = json.loads(subprocess.check_output([executable,"--map",estimate,reference],text=True))
        assert math.isclose(out["occupied_iou"],1/3) and out["evaluated_cells"]==3


if __name__=="__main__":
    main(sys.argv[1])
