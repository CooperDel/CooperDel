import * as THREE from 'three';
const V=(x=0,y=0,z=0)=>new THREE.Vector3(x,y,z);
export function createRoboNavSuspension(robot,k){
  const left=k.makeControl('Left Drivetrain Integration Revolute',V(0,0,1));
  const right=k.makeControl('Right Drivetrain Integration Revolute',V(0,0,1));
  const bar=k.makeControl('Diff Bar Revolute',V(1,0,0));
  const connectors=k.differential.map(d=>({...d,start:V(d.end.x,0,0),length:V(d.end.x,0,0).distanceTo(d.end)}));
  let state=[0,0,0,0,0,0],contactErrors=[0,0,0,0];
  function reset(){left.set(0);right.set(0);bar.set(0);for(const c of connectors){c.link.position.set(0,0,0);c.link.quaternion.identity();}state.fill(0);contactErrors.fill(0);robot.updateMatrixWorld(true);}
  function solve(heightAt){
    reset();
    const baseQ=robot.quaternion.clone(),baseP=robot.position.clone();
    const reference=left.j.getWorldPosition(V()).add(right.j.getWorldPosition(V())).multiplyScalar(.5),local=robot.worldToLocal(reference.clone());
    const forward=V(1,0,0).applyQuaternion(baseQ),up=V(0,1,0),side=up.clone().cross(forward).normalize();
    function apply(q){
      robot.quaternion.copy(new THREE.Quaternion().setFromAxisAngle(side,q[1])).multiply(new THREE.Quaternion().setFromAxisAngle(forward,q[2])).multiply(baseQ);
      robot.position.copy(reference).sub(local.clone().applyQuaternion(robot.quaternion)).add(V(0,q[0],0));
      left.set(q[3]);right.set(q[4]);bar.set(q[5]);robot.updateMatrixWorld(true);
    }
    function residual(q){
      apply(q);
      const out=k.wheels.map(w=>{const p=w.link.localToWorld(w.center.clone());return p.y-w.radius-heightAt(p.x,p.z);});
      for(const c of connectors)out.push(c.link.localToWorld(c.start.clone()).distanceTo(c.bar.localToWorld(c.anchor.clone()))-c.length);
      return out;
    }
    for(let iteration=0;iteration<14;iteration++){
      const r=residual(state);if(Math.max(...r.map(Math.abs))<.000015)break;
      const jac=state.map((_,i)=>{const q=state.slice();q[i]+=.00002;return residual(q).map((v,j)=>(v-r[j])/.00002);});
      const a=state.map((_,i)=>[...state.map((_,j)=>jac[i].reduce((s,v,n)=>s+v*jac[j][n],0)+(i===j?1e-8:0)),-jac[i].reduce((s,v,n)=>s+v*r[n],0)]);
      for(let i=0;i<6;i++){let pivot=i;for(let j=i+1;j<6;j++)if(Math.abs(a[j][i])>Math.abs(a[pivot][i]))pivot=j;[a[i],a[pivot]]=[a[pivot],a[i]];const d=a[i][i];for(let j=i;j<=6;j++)a[i][j]/=d;for(let row=0;row<6;row++)if(row!==i){const f=a[row][i];for(let col=i;col<=6;col++)a[row][col]-=f*a[i][col];}}
      state=state.map((v,i)=>THREE.MathUtils.clamp(v+THREE.MathUtils.clamp(a[i][6],-.045,.045),i===0?-.15:-.4,i===0?.2:.4));
    }
    contactErrors=residual(state).slice(0,4);
    for(const c of connectors){const target=c.link.worldToLocal(c.bar.localToWorld(c.anchor.clone())).sub(c.start);const rotation=new THREE.Quaternion().setFromUnitVectors(c.end.clone().sub(c.start).normalize(),target.normalize());c.link.quaternion.copy(rotation);c.link.position.copy(c.start).sub(c.start.clone().applyQuaternion(rotation));}
    robot.updateMatrixWorld(true);
    if(!state.every(Number.isFinite)){robot.position.copy(baseP);robot.quaternion.copy(baseQ);reset();throw new Error('Invalid suspension pose');}
  }
  return {reset,solve,diagnostics:()=>({state:state.slice(),contactErrors:contactErrors.slice(),closureErrors:connectors.map(c=>c.link.localToWorld(c.end.clone()).distanceTo(c.bar.localToWorld(c.anchor.clone())))})};
}
