import * as THREE from 'three';
const V=(x=0,y=0,z=0)=>new THREE.Vector3(x,y,z);
function bounds(link){
 const box=new THREE.Box3();link.traverse(c=>{if(c.isURDFVisual)c.traverse(mesh=>{if(mesh.isMesh){mesh.geometry.computeBoundingBox();box.union(mesh.geometry.boundingBox);}});});return box;
}
export function createV3Suspension(robot,heightAt){
 const joint=name=>robot.joints[`${name} Revolute`] || robot.joints[`${name} Continuous`];
 // The rear-left STL is identical to the front-left module, but its exported frame is incorrect.
 const front=joint('Front Left Swerve'),rear=joint('Back Left Swerve');
 rear.position.set(.01995,.19393,-.015575);rear.quaternion.copy(front.quaternion);rear.origPosition=null;rear.origQuaternion=null;
 const rearWheel=joint('Back Left Wheel'),frontWheel=joint('Front Left Wheel');
 rearWheel.position.copy(frontWheel.position);rearWheel.quaternion.copy(frontWheel.quaternion);rearWheel.axis.copy(frontWheel.axis);rearWheel.origPosition=null;rearWheel.origQuaternion=null;
 robot.updateMatrixWorld(true);
 const worldAxis=(j,axis)=>{j.axis.copy(axis).applyQuaternion(j.getWorldQuaternion(new THREE.Quaternion()).invert()).normalize();j.limit.lower=-.5;j.limit.upper=.5;};
 const rockers=[joint('Left Drivetrain'),joint('Right Drivetrain')],bar=joint('Diff Bar');
 rockers.forEach(j=>worldAxis(j,V(0,0,1)));worldAxis(bar,V(1,0,0));
 const modules=['Front Left','Back Left','Front Right','Back Right'].map(name=>{
  const steering=joint(`${name} Swerve`),wheel=joint(`${name} Wheel`),link=robot.links[`${name} Wheel`];
  worldAxis(steering,V(0,1,0));
  if(name.endsWith('Right')){
   // Bake the inward-facing half-turn into the neutral mounting frame. Steering
   // commands stay relative to this frame, so reset and crab motion retain it.
   steering.quaternion.multiply(new THREE.Quaternion().setFromAxisAngle(steering.axis,Math.PI));
   steering.origQuaternion=null;steering.origPosition=null;
   steering.updateWorldMatrix(true,true);
  }
  steering.limit.lower=-Math.PI/2;steering.limit.upper=Math.PI/2;
  const b=bounds(link),center=b.getCenter(V());
  // Put the wheel's origin on its geometric axle center and keep the visual in place.
  const centerInJoint=link.localToWorld(center.clone());wheel.worldToLocal(centerInJoint);
  wheel.position.add(centerInJoint.clone().applyQuaternion(wheel.quaternion));
  link.position.sub(centerInJoint);wheel.origPosition=null;wheel.origQuaternion=null;
  const radius=(b.max.y-b.min.y)/2;
  const spinSign=Math.sign(wheel.axis.clone().transformDirection(wheel.matrixWorld).cross(V(0,1,0)).dot(V(1,0,0))) || 1;
  return {wheel,steering,link,center,radius,spinSign,spin:0};
 });
 robot.updateMatrixWorld(true);
 const centerOf=m=>m.link.localToWorld(m.center.clone());
 const basePosition=robot.position.clone(),baseQuaternion=robot.quaternion.clone();
 const reference=rockers[0].getWorldPosition(V()).add(rockers[1].getWorldPosition(V())).multiplyScalar(.5);
 const localReference=robot.worldToLocal(reference.clone());
 modules.forEach(m=>{m.offset=m.steering.getWorldPosition(V()).sub(reference);});
 const connectors=['Left','Right'].map(name=>{
  const j=joint(`${name} Link`),link=robot.links[`${name} Link`],c=bounds(link).getCenter(V());
  const start=V(c.x,0,0),end=V(c.x,2*c.y,2*c.z);
  const anchor=bar.worldToLocal(link.localToWorld(end.clone()));
  return {j,link,start,end,anchor,length:start.distanceTo(end)};
 });
 let state=[0,0,0,0,0,0],pathX=reference.x,pathZ=reference.z,heading=0,maxError=0;
 const initialHeight=modules.reduce((sum,m)=>sum+centerOf(m).y-m.radius,0)/4-reference.y;
 state[0]=-initialHeight;
 function apply(s){
  robot.quaternion.copy(new THREE.Quaternion().setFromEuler(new THREE.Euler(s[2],heading,s[1],'YXZ'))).multiply(baseQuaternion);
  robot.position.set(pathX,s[0],pathZ).sub(localReference.clone().applyQuaternion(robot.quaternion));
  rockers[0].setJointValue(s[3]);rockers[1].setJointValue(s[4]);bar.setJointValue(s[5]);
  // Residuals update only the joint chains they read; visual meshes update once after solving.
  robot.updateWorldMatrix(true,false);
 }
 function residual(s){
  apply(s);
  const errors=modules.map(m=>{const p=centerOf(m);return p.y-m.radius-heightAt(p.x,p.z);});
  connectors.forEach(c=>errors.push(c.j.localToWorld(c.start.clone()).distanceTo(bar.localToWorld(c.anchor.clone()))-c.length));return errors;
 }
 function solve(){
  for(let iteration=0;iteration<18;iteration++){
   const r=residual(state);if(Math.max(...r.map(Math.abs))<1e-6)break;
   const jac=state.map((_,i)=>{const s=state.slice();s[i]+=1e-5;return residual(s).map((n,k)=>(n-r[k])/1e-5);});
   const a=state.map((_,i)=>[...state.map((_,j)=>jac[i].reduce((sum,n,k)=>sum+n*jac[j][k],0)+(i===j?1e-8:0)),-jac[i].reduce((sum,n,k)=>sum+n*r[k],0)]);
   for(let i=0;i<6;i++){
    let pivot=i;for(let j=i+1;j<6;j++)if(Math.abs(a[j][i])>Math.abs(a[pivot][i]))pivot=j;
    [a[i],a[pivot]]=[a[pivot],a[i]];const d=a[i][i];for(let k=i;k<=6;k++)a[i][k]/=d;
    for(let j=0;j<6;j++)if(j!==i){const f=a[j][i];for(let k=i;k<=6;k++)a[j][k]-=f*a[i][k];}
   }
   state=state.map((n,i)=>i===0?n+THREE.MathUtils.clamp(a[i][6],-.03,.03):THREE.MathUtils.clamp(n+THREE.MathUtils.clamp(a[i][6],-.04,.04),-.45,.45));
  }
  apply(state);
  for(const c of connectors){
   const target=c.j.worldToLocal(bar.localToWorld(c.anchor.clone())).sub(c.start);
   c.link.quaternion.setFromUnitVectors(c.end.clone().sub(c.start).normalize(),target.normalize());
   c.link.position.copy(c.start).sub(c.start.clone().applyQuaternion(c.link.quaternion));
  }
  robot.updateMatrixWorld(true);
  maxError=Math.max(...connectors.map(c=>c.link.localToWorld(c.end.clone()).distanceTo(bar.localToWorld(c.anchor.clone()))));
  if(maxError>2e-4)throw new Error(`V3 linkage closure exceeds tolerance: ${(maxError*1000).toFixed(3)} mm`);
 }
 return {
  pose(x,z,yaw=0,travel=0,drive=null){
   pathX=x;pathZ=z;heading=yaw;
   modules.forEach(m=>{
    if(drive){
     const dx=m.offset.x*Math.cos(yaw)+m.offset.z*Math.sin(yaw),dz=-m.offset.x*Math.sin(yaw)+m.offset.z*Math.cos(yaw);
     const vx=drive.vx+drive.yawRate*dz,vz=drive.vz-drive.yawRate*dx;
     let angle=THREE.MathUtils.euclideanModulo(Math.atan2(vz,-vx)-yaw+Math.PI,2*Math.PI)-Math.PI,speed=Math.hypot(vx,vz);
     if(angle>Math.PI/2){angle-=Math.PI;speed=-speed;}else if(angle<-Math.PI/2){angle+=Math.PI;speed=-speed;}
     m.steering.setJointValue(angle);m.spin-=speed/m.radius*m.spinSign*drive.delta;m.wheel.setJointValue(m.spin);
    }else{m.steering.setJointValue(0);m.wheel.setJointValue(travel/m.radius*m.spinSign);}
   });solve();
  },
  resetDrive(){modules.forEach(m=>{m.spin=0;});},
  diagnostics(){return {closureErrors:connectors.map(c=>c.link.localToWorld(c.end.clone()).distanceTo(bar.localToWorld(c.anchor.clone()))),contactErrors:residual(state).slice(0,4),state:state.slice(),steering:modules.map(m=>m.steering.jointValue[0]),wheelAngles:modules.map(m=>m.wheel.jointValue[0])};},
  restore(){robot.position.copy(basePosition);robot.quaternion.copy(baseQuaternion);}
 };
}
