#!/usr/bin/env python3
"""J2: ONE trial = the frozen J1 v2 trial bring-up (j1_run_trial.py from SCI_J1_SCHEDULERS_V2_20261113, copied with the
declared edits in tests/j2_run_trial_edits.py) with exactly these changes:
  1. the C5.2 bridge starts through j2_bridge.py (the frozen J1 launcher + the J2 link impairment);
  2. the bridge also receives SCI_J2_CONDITION, SCI_J2_IMPAIR (JSON) and SCI_J2_SEED from spec['j2'].
Everything else is J1 v2 (E7 bring-up, E5 trial process, AMCL tolerance 1.0 s, J1 ledger and MAKL decision files).
Usage: j2_run_trial.py TRIAL_SPEC_JSON OUTPUT_DIR"""
import json,os,signal,subprocess,sys,time
from pathlib import Path
import yaml
PJ=Path(__file__).resolve().parent;P7=Path.home()/'ros2_ws/SCI_E7_QUEUE_DISCIPLINE_20261102';P5=Path.home()/'ros2_ws/SCI_E5_PED_SPEED_20261101'
ROOT=Path.home()/'ros2_ws/SCI_E3B_EDGE_NAV_RUNNER_20261031'
sys.path.insert(0,str(ROOT))
import run_trial as RT
from run_trial import sha,req,S2
from generate_config import generate
from generate_movement_config import generate as generate_movement
PORT=RT.PORT;DOMAIN=RT.DOMAIN
def apply_amcl_tol1(cfg,output):
    p=Path(cfg['params_file']);d=yaml.safe_load(p.read_text());a=d['amcl']['ros__parameters']
    req(abs(float(a['transform_tolerance'])-0.05)<1e-9,'E5_UNEXPECTED_AMCL_TOLERANCE')
    a['transform_tolerance']=1.0;p.write_text(yaml.safe_dump(d,sort_keys=False))
    (Path(output)/'E5_AMCL_TOL.json').write_text(json.dumps({'amcl_transform_tolerance':1.0,'params_sha256':sha(p)})+'\n')
    return dict(cfg,params_sha256=sha(p))

def main():
    spec=json.loads(sys.argv[1]);output=Path(sys.argv[2]);output.mkdir(parents=True,exist_ok=False)
    RT.gates(spec)
    req(isinstance(spec.get('e5'),dict),'E5_SPEC_MISSING')
    req(isinstance(spec.get('j1'),dict) and spec['j1'].get('config') in ('B0','B1','B2','B3','B4','B5','B7','MAKL'),'J1_CONFIG_MISSING')
    req(isinstance(spec['j1'].get('plan'),dict),'J1_PLAN_MISSING')
    req(isinstance(spec.get('j2'),dict) and isinstance(spec['j2'].get('impair'),dict) and isinstance(spec['j2'].get('seed'),int),'J2_IMPAIR_MISSING')
    env=os.environ.copy();env.update({'ROS_DOMAIN_ID':str(DOMAIN),'GAZEBO_MASTER_URI':f'http://127.0.0.1:{PORT}',
        'ROS_LOCALHOST_ONLY':'0','TURTLEBOT3_MODEL':'burger','PYTHONNOUSERSITE':'1','RMW_IMPLEMENTATION':'rmw_cyclonedds_cpp',
        'CYCLONEDDS_URI':(ROOT/'cyclonedds_e2.xml').resolve().as_uri()})
    import e3b_policy_logic as L3
    benv=dict(env,SCI_SIM_ONLY='YES',SCI_D3_OWNED_EXCLUSIVE='YES',SCI_D3_SCOPE_TOKEN=spec['scope_token'],
              SCI_D3_SCHEDULES_JSON=json.dumps([spec['j1']['plan']]),
              SCI_J1_CONFIG=spec['j1']['config'],SCI_J1_LEDGER=str(output/'J1_LEDGER.jsonl'),SCI_J1_DECISIONS=str(output/'J1_DECISIONS.jsonl'),
              SCI_J2_CONDITION=spec['j2']['condition'],SCI_J2_IMPAIR=json.dumps(spec['j2']['impair'],sort_keys=True),SCI_J2_SEED=str(spec['j2']['seed']))
    processes=[];rc=3
    handlers={s:signal.getsignal(s) for s in (signal.SIGINT,signal.SIGTERM)}
    def interrupted(sig,frame):raise RuntimeError('INTERRUPTED_'+str(sig))
    for s in handlers:signal.signal(s,interrupted)
    def start(name,cmd,e=None):
        with (output/(name+'.log')).open('wb') as f:
            proc=subprocess.Popen(cmd,env=e or env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append((name,proc));return proc
    def run(name,cmd,timeout):
        log=output/(name+'.log')
        with log.open('wb') as f:
            proc=subprocess.Popen(cmd,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+timeout
            while proc.poll() is None and time.monotonic()<deadline:
                for owned_name,owned in processes:
                    if owned.poll() is not None:
                        RT.terminate(proc)
                        if processes:RT.terminate(processes[0][1])
                        raise RuntimeError(name+'_OWNED_PROCESS_DIED_'+owned_name+'_'+str(owned.returncode))
                time.sleep(.12)
            if proc.poll() is None:
                RT.terminate(proc);RT.terminate(processes[0][1]) if processes else None
                raise RuntimeError(name+'_TIMEOUT')
            req(proc.returncode==0 or name=='e3b_trial',name+'_RETURN_'+str(proc.returncode))
            return proc.returncode
    phase='setup'
    try:
        (output/'TRIAL_SPEC.json').write_text(json.dumps({k:v for k,v in spec.items() if k!='scope_token'},indent=2)+'\n')
        run('dds_capacity_preflight',['/usr/bin/python3','-u',str(ROOT/'dds_capacity_preflight.py'),str(output/'DDS_PREFLIGHT')],102)
        run('final_guard_init_preflight',['/usr/bin/python3','-u',str(ROOT/'final_guard_init_preflight.py')],25)
        start('gazebo',['gzserver','--lockstep',str(ROOT/'reference/train_00_e2_realtime.world'),'-s','libgazebo_ros_init.so','-s','libgazebo_ros_factory.so'])
        time.sleep(2);req(processes[0][1].poll() is None,'PRIVATE_GAZEBO_DID_NOT_START')
        run('stage1_robot_probe',['/usr/bin/python3','-u',str(ROOT/'robot_probe_camera.py'),str(output/'ROBOT_ATTESTATION.json')],135)
        attest=json.loads((output/'ROBOT_ATTESTATION.json').read_text())
        req(attest['status']=='STAGE1_ROBOT_SENSOR_ATTESTED_NO_COMMANDED_MOTION','STAGE1_NOT_ATTESTED')
        cfg=generate(output,edge=True)     # E3-B: global costmap on scan_edge; AMCL stays on raw /scan
        cfg=apply_amcl_tol1(cfg,output)    # P0-frozen fix
        from prepare_frames import prepare
        frames=prepare(output,attest['robot_sdf'])
        start('robot_state_publisher',['ros2','run','robot_state_publisher','robot_state_publisher','--ros-args','--params-file',frames['rsp_params_file']])
        run('tf_clock_probe',['/usr/bin/python3','-u',str(ROOT/'tf_clock_probe.py'),str(output/'TF_CLOCK_ATTESTATION.json')],42)
        start('map_server',['ros2','run','nav2_map_server','map_server','--ros-args','--params-file',cfg['params_file']])
        start('amcl',['ros2','run','nav2_amcl','amcl','--ros-args','--params-file',cfg['params_file']])
        time.sleep(1.5)
        start('localization_lifecycle',['ros2','run','nav2_lifecycle_manager','lifecycle_manager','--ros-args','-r','__node:=lifecycle_manager_e2_stage3_localization','--params-file',cfg['localization_lifecycle_file']])
        run('stationary_amcl_probe',['/usr/bin/python3','-u',str(ROOT/'amcl_probe.py'),str(output/'AMCL_ATTESTATION.json')],100)
        req(json.loads((output/'AMCL_ATTESTATION.json').read_text())['status']=='STAGE3_AMCL_STATIONARY_POSE_AND_TF_PASS','AMCL_RESULT_FAIL')
        start('planner_server',['ros2','run','nav2_planner','planner_server','--ros-args','--params-file',cfg['params_file']])
        time.sleep(1.5)
        start('planner_lifecycle',['ros2','run','nav2_lifecycle_manager','lifecycle_manager','--ros-args','-r','__node:=lifecycle_manager_e2_stage3_planner','--params-file',cfg['planner_lifecycle_file']])
        run('recheck_nav2_path',['/usr/bin/python3','-u',str(S2/'planner_probe.py'),str(output/'NAV2_PATH_RESULT.json')],110)
        req(json.loads((output/'NAV2_PATH_RESULT.json').read_text())['status']=='STAGE2_ISOLATED_NAV2_PATH_ATTESTED_NO_MOTION','NAV2_PATH_FAILED')
        movement=generate_movement(output,edge=True)   # E3-B: local costmap on scan_edge; collision monitor on raw /scan
        (output/'MOVEMENT_CHAIN_CONFIG.json').write_text(json.dumps(movement,indent=2)+'\n')
        start('c52_joint_bridge',['/usr/bin/python3','-u',str(PJ/'j2_bridge.py'),
              '--mode','Q2','--capacity-mbps','3.0','--queue-kb','768','--report-sim-s','5'],benv)
        start('e2_mode_gate',['/usr/bin/python3','-u',str(ROOT/'e2_mode_gate.py')])
        start('final_hard_guard',['/usr/bin/python3','-u',str(ROOT/'safety_chain.py')])
        start('collision_monitor',['ros2','run','nav2_collision_monitor','collision_monitor','--ros-args','--params-file',movement['collision_monitor']])
        start('controller_server',['ros2','run','nav2_controller','controller_server','--ros-args','--params-file',movement['controller'],'-r','cmd_vel:=/e2_nav_cmd_policy'])
        time.sleep(2)
        start('movement_lifecycle',['ros2','run','nav2_lifecycle_manager','lifecycle_manager','--ros-args','-r','__node:=lifecycle_manager_e2_stage4_control','--params-file',movement['controller_lifecycle']])
        phase='trial'
        run('e3b_trial',['/usr/bin/python3','-u',str(P5/'e5_trial.py'),str(output/'TRIAL_RESULT.json'),json.dumps(spec)],600)
        rc=0
    except Exception as e:
        (output/'ERROR.txt').write_text(repr(e)+'\n')
        print('E2_RUN_TRIAL_FAIL_CLOSED',phase,repr(e),flush=True)
    finally:
        for sig,h in handlers.items():signal.signal(sig,h)
        if processes:RT.terminate(processes[0][1])      # Gazebo first: no continuing motion
        for name,proc in reversed(processes[1:]):RT.terminate(proc)
        (output/'OWNED_CHILDREN_TERMINATED.txt').write_text('\n'.join(f'{n}: {p.poll()}' for n,p in processes)+'\n')
        res=output/'TRIAL_RESULT.json'
        if not res.is_file():
            moved=(output/'GOAL_ACCEPTED.marker').is_file()
            res.write_text(json.dumps({'trial_id':spec['trial_id'],'status':'BLOCKED' if moved else 'PRE_MOTION_BLOCKED',
                'error':(output/'ERROR.txt').read_text().strip() if (output/'ERROR.txt').is_file() else 'UNKNOWN','phase':phase},indent=2)+'\n')
        try:   # costmap message-filter drops (late scans that could not be transformed), from the Nav2 logs
            r=json.loads(res.read_text())
            drops={n:sum(l.count('Message Filter dropping message') for l in (output/(n+'.log')).read_text(errors='replace').splitlines())
                   for n in ('controller_server','planner_server') if (output/(n+'.log')).is_file()}
            r['costmap_filter_drops']=drops
            r['costmap_filter_drop_ratio']=(drops.get('controller_server',0)/r['edge_scans_received']) if r.get('edge_scans_received') else None
            res.write_text(json.dumps(r,indent=2,default=str)+'\n')
        except Exception as ex:(output/'FILTER_DROP_COUNT_ERROR.txt').write_text(repr(ex)+'\n')
        print('E2_RUN_TRIAL_DONE',spec['trial_id'],json.loads(res.read_text()).get('status'),flush=True)
    return rc
if __name__=='__main__':
    try:sys.exit(main())
    except Exception as e:print('J2_RUN_TRIAL_SETUP_BLOCKED',repr(e),flush=True);sys.exit(3)
