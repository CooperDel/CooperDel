import * as THREE from 'three';
const V=(x=0,y=0,z=0)=>new THREE.Vector3(x,y,z);
const wrap=a=>Math.atan2(Math.sin(a),Math.cos(a));
// URDFLoader wraps meshes in URDFVisual groups. Only visit this link's visuals;
// descending through its joints would include other links in its measurements.
export function robonavVisualMeshes(link){
  const meshes=[];
  for(const child of link.children){
    if(child.isURDFJoint||child.isURDFLink||child.isURDFCollider)continue;
    child.traverse(object=>{if(object.isMesh)meshes.push(object);});
  }
  return meshes;
}
export function createRoboNavKinematics(robot){
  const joint=name=>robot.joints[name];
  const world=o=>o.getWorldPosition(V());
  const bounds=name=>{
    const link=robot.links[name];link.updateWorldMatrix(true,true);
    const inverse=link.matrixWorld.clone().invert(),b=new THREE.Box3();
    for(const mesh of robonavVisualMeshes(link)){
      mesh.geometry.computeBoundingBox();
      b.union(mesh.geometry.boundingBox.clone().applyMatrix4(new THREE.Matrix4().multiplyMatrices(inverse,mesh.matrixWorld)));
    }
    if(b.isEmpty()||![...b.min.toArray(),...b.max.toArray()].every(Number.isFinite))throw new Error(`Missing or invalid RoboNav visual geometry: ${name}`);
    return b;
  };
  // Repair the exported inward-facing claw. Both hinge frames now share the
  // wrist's transverse plane; opposite signed angles produce symmetric closure.
  for(const side of ['Left','Right'])joint(`${side} Claw Revolute`).quaternion.setFromEuler(new THREE.Euler(0,-Math.PI/2,0,'ZYX'));
  robot.updateMatrixWorld(true);
  const wrist=robot.links['Diff Wrist Output'];
  // The bottom front link was exported in assembly coordinates. Rebase its
  // pivot to the matching pin while preserving every vertex in world space.
  const bottom=joint('Bottom Front Linkages Revolute'),bottomLink=robot.links['Bottom Front Linkages'];
  const old=bottomLink.matrixWorld.clone();bottom.position.set(.126084,-.026,-.0295);bottom.quaternion.copy(joint('Top Front Linkages Revolute').quaternion);robot.updateMatrixWorld(true);
  const correction=bottomLink.matrixWorld.clone().invert().multiply(old);
  for(const c of bottomLink.children)if(!c.isURDFJoint&&!c.isURDFLink&&!c.isURDFCollider)c.applyMatrix4(correction);
  const controls=new Map();
  function control(name,axisWorld){
    const j=joint(name);robot.updateMatrixWorld(true);
    j.axis.copy(axisWorld).applyQuaternion(j.getWorldQuaternion(new THREE.Quaternion()).invert()).normalize();
    j.origPosition=j.position.clone();j.origQuaternion=j.quaternion.clone();j.limit.lower=-Math.PI*2;j.limit.upper=Math.PI*2;j.ignoreLimits=true;
    const c={j,rest:j.quaternion.clone(),set(a){j.quaternion.copy(this.rest).multiply(new THREE.Quaternion().setFromAxisAngle(j.axis,a));j.jointValue[0]=a;j.matrixWorldNeedsUpdate=true;},get value(){return j.jointValue[0]||0;}};
    controls.set(name,c);return c;
  }
  const arm=['Arm Bicep Revolute','Arm Forearm Revolute','Diff Wrist Pitch'].map(n=>control(n,V(0,0,1)));
  // Deploy the tower and pan at the imported gimbal pivot; preserve the camera hinge.
  const tower=control('Upper Tower Hinge',V(1,0,0));
  // Pan at the imported pivot; leave the upper camera hinge untouched.
  const gimbal=control('Gimbal Dynamic Revolute',V(0,-1,0));
  const wristAxis=V(0,1,0).transformDirection(wrist.matrixWorld);
  const clawAxis=V(0,0,1).transformDirection(wrist.matrixWorld);
  const claws=['Left','Right'].map(side=>control(`${side} Claw Revolute`,clawAxis));
  const pointIn=(link,p)=>link.worldToLocal(p.clone());
  const fingerLoops=['Top','Bottom'].map((side,index)=>{
    const back=control(`${side} Back Linkage Revolute`,wristAxis),front=control(`${side} Front Linkages Revolute`,wristAxis),finger=control(`${side} Finger Hinge`,wristAxis),mid=control(`${side} Mid Linkage Revolute`,wristAxis),connector=control(`${side} Mid Connector Revolute`,wristAxis);
    robot.updateMatrixWorld(true);
    const backLink=robot.links[`${side} Back Linkage`],fingerLink=robot.links[`${side} Finger`],frontLink=robot.links[`${side} Front Linkages`],midLink=robot.links[`${side} Mid Linkage`],connectorLink=robot.links[`${side} Mid Connector`];
    const fingerPin=world(finger.j).add(V(1,0,0).transformDirection(wrist.matrixWorld).multiplyScalar(.075));
    // The finger and front linkage share the measured 75 mm pin spacing.
    const frontEnd=pointIn(frontLink,fingerPin),fingerEnd=pointIn(fingerLink,fingerPin);
    const cb=bounds(`${side} Mid Connector`),distal=V(.005,cb.max.y-.005,cb.max.z-.005);
    const distalWorld=connectorLink.localToWorld(distal.clone());
    const backEnd=pointIn(backLink,distalWorld);
    const shoulder=pointIn(wrist,world(mid.j)),elbow=pointIn(wrist,world(connector.j)),end=pointIn(wrist,distalWorld);
    const l1=Math.hypot(elbow.x-shoulder.x,elbow.z-shoulder.z),l2=Math.hypot(end.x-elbow.x,end.z-elbow.z);
    const a1=Math.atan2(elbow.z-shoulder.z,elbow.x-shoulder.x),a2=Math.atan2(end.z-elbow.z,end.x-elbow.x);
    const cross=(elbow.x-shoulder.x)*(end.z-shoulder.z)-(elbow.z-shoulder.z)*(end.x-shoulder.x);
    return {side,index,back,front,finger,mid,connector,backLink,fingerLink,frontLink,connectorLink,frontEnd,fingerEnd,backEnd,distal,shoulder,l1,l2,a1,a2,branch:Math.sign(cross)||1};
  });
  function fingers(amount){
    for(const f of fingerLoops){
      const q=(f.index===0?1:-1)*amount;
      f.back.set(q);f.front.set(q);f.finger.set(-q);robot.updateMatrixWorld(true);
      const end=pointIn(wrist,f.backLink.localToWorld(f.backEnd.clone()));
      const dx=end.x-f.shoulder.x,dz=end.z-f.shoulder.z,d=Math.hypot(dx,dz);
      const along=(f.l1*f.l1-f.l2*f.l2+d*d)/(2*d),height=Math.sqrt(Math.max(0,f.l1*f.l1-along*along));
      const ex=f.shoulder.x+along*dx/d+f.branch*height*dz/d,ez=f.shoulder.z+along*dz/d-f.branch*height*dx/d;
      const angle1=Math.atan2(ez-f.shoulder.z,ex-f.shoulder.x),angle2=Math.atan2(end.z-ez,end.x-ex);
      // Positive rotation around wrist Y decreases the XZ polar angle.
      const m=wrap(f.a1-angle1);f.mid.set(m);f.connector.set(wrap(f.a2-angle2)-m);
    }
    robot.updateMatrixWorld(true);
  }
  function grip(clawAmount,fingerAmount=clawAmount){claws[0].set(-clawAmount*.89);claws[1].set(clawAmount*.89);fingers(fingerAmount*.85);}
  const toolLocal=V(.199,0,.0005);
  robot.updateMatrixWorld(true);
  const shoulder0=world(arm[0].j),elbow0=world(arm[1].j),pitch0=world(arm[2].j),tool0=wrist.localToWorld(toolLocal.clone());
  const a1=Math.atan2(elbow0.y-shoulder0.y,elbow0.x-shoulder0.x),a2=Math.atan2(pitch0.y-elbow0.y,pitch0.x-elbow0.x);
  const length1=shoulder0.distanceTo(elbow0),length2=Math.hypot(pitch0.x-elbow0.x,pitch0.y-elbow0.y);
  const wristRestQuaternion=wrist.getWorldQuaternion(new THREE.Quaternion());
  const forward=V(1,0,0).transformDirection(wrist.matrixWorld),orientation0=Math.atan2(forward.y,forward.x),offset=tool0.clone().sub(pitch0);
  function solveArm(target,angle,localPoint=toolLocal){
    const shoulder=world(arm[0].j),delta=angle-orientation0;
    const rotated=offset.clone().add(localPoint.clone().sub(toolLocal).applyQuaternion(wristRestQuaternion)).applyAxisAngle(V(0,0,1),delta);
    const end=target.clone().sub(rotated),dx=end.x-shoulder.x,dy=end.y-shoulder.y,d=Math.hypot(dx,dy);
    const c=THREE.MathUtils.clamp((length1*length1-length2*length2+d*d)/(2*length1*d),-1,1);
    const theta=Math.atan2(dy,dx)+Math.acos(c);
    const ex=shoulder.x+length1*Math.cos(theta),ey=shoulder.y+length1*Math.sin(theta);
    const theta2=Math.atan2(end.y-ey,end.x-ex);
    let q1=wrap(theta-a1);if(q1>0)q1-=Math.PI*2;
    const q2=wrap(theta2-a2-q1),q3=delta-q1-q2;
    return [q1,q2,q3];
  }
  function poseArm(q){arm.forEach((c,i)=>c.set(q[i]));robot.updateMatrixWorld(true);}
  const wheels=['Front Left','Back Left','Front Right','Back Right'].map(n=>{
    const j=joint(`${n} Wheel Revolute`),link=robot.links[`${n} Wheel`],b=bounds(`${n} Wheel`),center=b.getCenter(V());
    // Wheel meshes are centered on their local X axle, including the tiny CAD offset.
    const c=control(`${n} Wheel Revolute`,V(0,0,-1));
    const steering=joint(`${n} Swerve Revolute`),steeringLink=robot.links[`${n} Swerve`];
    const oldWorld=steeringLink.matrixWorld.clone(),pivot=link.localToWorld(center.clone()).add(V(0,.12,0));
    steering.position.copy(steering.parent.worldToLocal(pivot));robot.updateMatrixWorld(true);
    steeringLink.applyMatrix4(steeringLink.matrixWorld.clone().invert().multiply(oldWorld));robot.updateMatrixWorld(true);
    const steer=control(`${n} Swerve Revolute`,V(0,1,0));
    return {c,steer,radius:(b.max.z-b.min.z)/2,link,center};
  });
  // Capture both bar-end anchors in the assembled rest pose. The suspension
  // solver uses these anchors to preserve the rigid differential loop.
  const differential=['Left','Right'].map(side=>{
    const link=robot.links[`${side} Diff Bar Link`],b=bounds(`${side} Diff Bar Link`),end=V((b.min.x+b.max.x)/2,0,side==='Left'?b.min.z+.0111:b.max.z-.0111);
    const bar=robot.links['Diff Bar'];return {link,end,bar,anchor:bar.worldToLocal(link.localToWorld(end.clone()))};
  });
  function closureError(){let error=0;for(const f of fingerLoops){error=Math.max(error,f.frontLink.localToWorld(f.frontEnd.clone()).distanceTo(f.fingerLink.localToWorld(f.fingerEnd.clone())),f.backLink.localToWorld(f.backEnd.clone()).distanceTo(f.connectorLink.localToWorld(f.distal.clone())));}for(const d of differential)error=Math.max(error,d.link.localToWorld(d.end.clone()).distanceTo(d.bar.localToWorld(d.anchor.clone())));return error;}
  grip(1);
  const pushLocal=V();let lowest=Infinity;
  // Use the lowest actual gripper vertex at the task's approach pitch as the
  // lid contact, so no other finger passes through the closed lid.
  for(const name of ['Right Claw','Left Claw','Top Finger','Bottom Finger'])for(const mesh of robonavVisualMeshes(robot.links[name])){const p=mesh.geometry.attributes.position;for(let i=0;i<p.count;i++){const v=wrist.worldToLocal(mesh.localToWorld(V().fromBufferAttribute(p,i)));const height=v.x*Math.sin(-1.2)+v.z*Math.cos(-1.2);if(height<lowest){lowest=height;pushLocal.copy(v);}}}
  grip(0);
  return {makeControl:control,differential,solveArm,poseArm,grip,pushLocal,tower,gimbal,wheels,toolLocal,fingerToolLocal:V(.199,0,-.0055),wrist,toolPosition:()=>wrist.localToWorld(toolLocal.clone()),orientation0,closureError,fingerLoops,claws,controls};
}
