import * as THREE from 'three';

// Illustrative force/torque allocation for the four steerable thrusters.
// Commands follow route acceleration, drag and angular motion; no fluid solver.
export function createUUVThrusters(robot,centerLocal,size){
  robot.updateMatrixWorld(true);
  const inverse=robot.getWorldQuaternion(new THREE.Quaternion()).invert();
  const names=['bottom_left_thruster_cw','bottom_right_thruster_ccw','top_left_thruster_ccw','top_right_thruster_cw'];
  const entries=names.map(name=>{
    const joint=robot.joints[`dof_${name}`];
    const axis=joint.axis.clone().transformDirection(joint.matrixWorld).applyQuaternion(inverse).normalize();
    const position=robot.worldToLocal(joint.getWorldPosition(new THREE.Vector3()));
    return {joint,initial:joint.jointValue[0]||0,axis,arm:position.sub(centerLocal).divideScalar(size),direction:new THREE.Vector3(0,1,0),throttle:0};
  });
  let values=new Array(8).fill(0);
  function wrench(q){
    const force=new THREE.Vector3(),torque=new THREE.Vector3();
    entries.forEach((e,i)=>{const thrust=new THREE.Vector3(0,1,0).applyAxisAngle(e.axis,q[i]).multiplyScalar(q[4+i]);force.add(thrust);torque.add(e.arm.clone().cross(thrust));});
    return [...force.toArray(),...torque.toArray()];
  }
  function solveLinear(A,b){
    for(let i=0;i<b.length;i++){
      let k=i;for(let j=i+1;j<b.length;j++)if(Math.abs(A[j][i])>Math.abs(A[k][i]))k=j;
      [A[i],A[k]]=[A[k],A[i]];[b[i],b[k]]=[b[k],b[i]];
      for(let j=i+1;j<b.length;j++){const f=A[j][i]/A[i][i];for(let c=i+1;c<b.length;c++)A[j][c]-=f*A[i][c];b[j]-=f*b[i];}
    }
    const x=new Array(b.length).fill(0);for(let i=b.length-1;i>=0;i--){let t=b[i];for(let j=i+1;j<b.length;j++)t-=A[i][j]*x[j];x[i]=t/A[i][i];}return x;
  }
  return {entries,
    update(force,torque,dt){
      const desired=[...force.toArray(),...torque.toArray()],prior=values.slice();
      for(let iteration=0;iteration<6;iteration++){
        const current=wrench(values),error=desired.map((v,i)=>v-current[i]);
        const J=values.map((v,i)=>{values[i]=v+1e-4;const next=wrench(values);values[i]=v;return next.map((n,k)=>(n-current[k])/1e-4);});
        const A=J.map((a,i)=>J.map((b,j)=>a.reduce((s,v,k)=>s+v*b[k],0)+(i===j?.001:0)));
        const b=J.map(a=>a.reduce((s,v,k)=>s+v*error[k],0));const step=solveLinear(A,b);
        values=values.map((v,i)=>THREE.MathUtils.clamp(v+THREE.MathUtils.clamp(step[i],-.3,.3),i<4?-1.05:-1,i<4?1.05:1));
      }
      values=values.map((v,i)=>prior[i]+THREE.MathUtils.clamp(v-prior[i],-dt*(i<4?1.6:2.5),dt*(i<4?1.6:2.5)));
      entries.forEach((e,i)=>{e.joint.setJointValue(e.initial+values[i]);e.direction.set(0,1,0).applyAxisAngle(e.axis,values[i]);e.throttle=values[i+4];});
    },
    reset(){values.fill(0);entries.forEach(e=>{e.joint.setJointValue(e.initial);e.direction.set(0,1,0);e.throttle=0;});},
    dispose(){entries.forEach(e=>e.joint.setJointValue(e.initial));}
  };
}
