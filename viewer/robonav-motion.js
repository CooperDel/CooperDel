import * as THREE from 'three';
import {createRoboNavSuspension} from './robonav-suspension.js?v=20261008-animation2';
import {createRoboNavKinematics} from './robonav-kinematics.js?v=20261008-animation2';
const V=(x=0,y=0,z=0)=>new THREE.Vector3(x,y,z);
const ease=x=>{x=THREE.MathUtils.clamp(x,0,1);return x*x*x*(10+x*(-15+6*x));};
const progress=(t,a,b)=>ease((t-a)/(b-a));
export const robonavCycleDuration=138;
export function createRoboNavMotion(robot,scene){
  const originalQuaternion=robot.quaternion.clone(),originalPosition=robot.position.clone(),originalBackground=scene.background,originalFog=scene.fog;
  const k=createRoboNavKinematics(robot);
  const suspension=createRoboNavSuspension(robot,k);
  robot.updateMatrixWorld(true);
  const wheelBottom=Math.min(...k.wheels.map(w=>w.link.localToWorld(w.center.clone()).y-w.radius));
  robot.position.y-=wheelBottom;robot.updateMatrixWorld(true);
  const base=robot.position.clone(),toolZ=k.toolPosition().z,travel=1.35;
  const stationX=travel+2.4,boxX=stationX+.24,boxWidth=.28,boxHeight=.15,boxDepth=.29;
  const roughZ=toolZ-.65;
  const bumps=[{x:.85,z:roughZ+.333,r:.25,h:.075},{x:.3,z:roughZ-.333,r:.27,h:.085},{x:-.25,z:roughZ+.333,r:.25,h:.065},{x:-.8,z:roughZ-.333,r:.24,h:.065}];
  const bumpHeight=(x,z)=>bumps.reduce((h,b)=>{const r=Math.hypot(x-b.x,z-b.z)/b.r;return h+(r<1?b.h*Math.cos(r*Math.PI/2)**2:0);},0);
  const groundTube=V(stationX,.052,toolZ),deposit=V(boxX,.078,toolZ);
  const environment=new THREE.Group();environment.name='Mars sample collection';scene.add(environment);
  scene.background=new THREE.Color(0x9a6a52);scene.fog=new THREE.Fog(0x9a6a52,8,19);
  const material=(color,extra={})=>new THREE.MeshStandardMaterial({color,roughness:.9,...extra});
  const soil=material(0x984b2e),rockMat=material(0x713b2a,{flatShading:true}),metal=material(0xd1bb8b,{roughness:.5,metalness:.3}),dark=material(0x292c2c);
  const make=(geometry,mat,parent=environment)=>{const mesh=new THREE.Mesh(geometry,mat);parent.add(mesh);return mesh;};
  const ground=make(new THREE.PlaneGeometry(28,22,90,70),soil);ground.rotation.x=-Math.PI/2;ground.position.set(3,-.003,-1);
  // Keep the wheel lane and sampling pad level; roughen the surrounding terrain.
  const gp=ground.geometry.attributes.position;
  for(let i=0;i<gp.count;i++){const x=gp.getX(i)+3,z=-gp.getY(i)-1;const lane=z>toolZ-2&&z<toolZ+.8&&x>-2.5&&x<4.9;gp.setZ(i,lane?0:.028*Math.sin(x*4.7+z*1.3)*Math.sin(z*5.1)+.013*Math.sin(x*13-z*7));}ground.geometry.computeVertexNormals();
  const roughGeometry=new THREE.PlaneGeometry(3.4,1.3,170,65);roughGeometry.rotateX(-Math.PI/2);const rough=make(roughGeometry,soil);rough.position.set(0,0,roughZ);
  const rp=roughGeometry.attributes.position;for(let i=0;i<rp.count;i++)rp.setY(i,bumpHeight(rp.getX(i),rp.getZ(i)+roughZ));roughGeometry.computeVertexNormals();
  let seed=17421;const random=()=>{seed=(1664525*seed+1013904223)>>>0;return seed/4294967296;};
  for(let i=0;i<105;i++){const x=-3+random()*13,z=-6+random()*10;if(x>-2.5&&x<4.9&&z>toolZ-2.15&&z<toolZ+.9)continue;const size=.025+random()*.17;const rock=make(new THREE.DodecahedronGeometry(size,0),rockMat);rock.position.set(x,size*.3,z);rock.scale.set(1,.45+random()*.4,.7+random()*.6);rock.rotation.set(random()*2,random()*6,random()*2);}
  for(let i=0;i<12;i++){const hill=make(new THREE.ConeGeometry(.9+random()*1.7,.4+random(),7),soil);hill.position.set(-6+i*1.7,-.12,-7-random()*2);hill.rotation.y=random()*6;}
  // Open sample box with a true hinge and four walls, rather than a solid cube.
  const box=new THREE.Group();box.position.set(boxX,0,toolZ);environment.add(box);
  const slab=(w,h,d,x,y,z)=>{const m=make(new THREE.BoxGeometry(w,h,d),metal,box);m.position.set(x,y,z);return m;};
  slab(boxWidth,.012,boxDepth,0,.006,0);
  slab(.009,boxHeight,boxDepth,-boxWidth/2,.075,0);slab(.009,boxHeight,boxDepth,boxWidth/2,.075,0);
  slab(boxWidth,boxHeight,.009,0,.075,-boxDepth/2);slab(boxWidth,boxHeight,.009,0,.075,boxDepth/2);
  const insert=make(new THREE.BoxGeometry(.065,.014,.06),dark,box);insert.position.set(0,.018,0);
  const lid=new THREE.Group();lid.position.set(boxWidth/2,boxHeight,0);box.add(lid);
  const lidPanel=make(new THREE.BoxGeometry(boxWidth,.012,boxDepth),metal,lid);lidPanel.position.x=-boxWidth/2;
  const stripe=make(new THREE.BoxGeometry(.1,.002,.018),dark,lid);stripe.position.set(-boxWidth/2,.007,0);
  // A transverse handle gives the two fingers opposing contact surfaces.
  const handle=make(new THREE.CylinderGeometry(.013,.013,.108,20),dark,lid);handle.position.set(-boxWidth/2,.063,0);
  const handleBar=make(new THREE.CylinderGeometry(.0245,.0245,.08,24),metal,lid);handleBar.rotation.x=Math.PI/2;handleBar.position.set(-boxWidth/2,.095,0);
  const hinge=make(new THREE.CylinderGeometry(.008,.008,boxDepth,16),dark,box);hinge.rotation.x=Math.PI/2;hinge.position.copy(lid.position);
  const tube=new THREE.Group();tube.name='Test tube';environment.add(tube);
  const glass=material(0xb6e3df,{transparent:true,opacity:.65,roughness:.18,metalness:.05,depthWrite:false});
  make(new THREE.CylinderGeometry(.013,.013,.088,20,1,true),glass,tube);
  const roundBottom=make(new THREE.SphereGeometry(.013,16,10),glass,tube);roundBottom.position.y=-.044;roundBottom.scale.y=.55;
  const sample=make(new THREE.CylinderGeometry(.01,.01,.037,16),material(0xb6502b),tube);sample.position.y=-.026;
  const cap=make(new THREE.CylinderGeometry(.014,.014,.013,20),material(0xe4bd60),tube);cap.position.y=.05;
  const label=make(new THREE.CylinderGeometry(.0132,.0132,.014,20,1,true),material(0xf0e3cb),tube);label.position.y=.018;
  // Tire marks remain fixed in the terrain and reveal as the rover advances.
  const tracks=[];
  for(const z of [toolZ-.333,toolZ+.333])for(let i=0;i<47;i++){const mark=make(new THREE.BoxGeometry(.016,.001,.064),material(0x743c29));mark.position.set(1.3+i*.04,.0001,z);mark.visible=false;tracks.push(mark);}
  let phase='Raising the antenna',lastTime=0,attachment=null,graspError=0,targetError=0;
  const ready=V(travel+2.36,1.04,toolZ),aboveTube=groundTube.clone().add(V(0,.22,0)),aboveBox=deposit.clone().setY(.32);
  const toolAngle=-1.2;
  const boxGrasp=V(boxX,.245,toolZ),aboveHandle=V(boxX,.51,toolZ),carry=V(boxX,.6,toolZ);
  const turnPivot=V(1.7072+travel,0,toolZ),boxHome=box.position.clone();
  // Calibrate with the rover parked at the pickup station (its 1.35 m translation).
  robot.position.copy(base).add(V(travel,0,0));robot.updateMatrixWorld(true);
  k.poseArm(k.solveArm(boxGrasp,toolAngle,k.fingerToolLocal));
  const boxAttachment=k.wrist.matrixWorld.clone().invert().multiply(new THREE.Matrix4().makeTranslation(...boxHome.toArray()));
  robot.position.copy(base);k.poseArm([0,0,0]);
  const wheelOffsets=k.wheels.map(w=>w.link.localToWorld(w.center.clone()).sub(V(1.7072,0,toolZ)));
  const steeringAngles=wheelOffsets.map(p=>{let a=Math.atan2(p.x,p.z);if(a>Math.PI/2)a-=Math.PI;if(a< -Math.PI/2)a+=Math.PI;return a;});
  const spinSigns=wheelOffsets.map((p,i)=>Math.sign(p.z*Math.cos(steeringAngles[i])+p.x*Math.sin(steeringAngles[i])));

  const contactZ=k.wrist.localToWorld(k.pushLocal.clone()).z-toolZ;
  const lidPoint=angle=>V(-boxWidth+.003,.009,contactZ).applyAxisAngle(V(0,0,1),angle).add(lid.position).add(box.position);
  const positionFor=p=>k.wrist.localToWorld(p.clone());
  function placeArm(target,point=k.toolLocal){k.poseArm(k.solveArm(target,toolAngle,point));targetError=positionFor(point).distanceTo(target);}
  function interpolate(a,b,t){return a.clone().lerp(b,t);}
  function apply(t){
    t=Math.max(0,t)%robonavCycleDuration;lastTime=t;
    suspension.reset();
    const drive=progress(t,6,15)*travel;
    k.tower.set(Math.PI*progress(t,0,3.5));
    robot.quaternion.copy(originalQuaternion);robot.position.copy(base).add(V(drive,0,0));
    box.position.copy(boxHome);box.quaternion.identity();
    const heading=Math.PI/2*(progress(t,69,75)+progress(t,100,106)),deliveryDistance=.65*progress(t,77,81),roughDistance=4.7*progress(t,108,130);
    k.gimbal.set(Math.PI/2*(progress(t,67,69)+progress(t,98,100))-heading);
    const steer=progress(t,67,69)*(1-progress(t,75,77))+progress(t,98,100)*(1-progress(t,106,108));
    for(const [i,w] of k.wheels.entries()){
      w.steer.set(steeringAngles[i]*steer);
      w.c.set((drive+spinSigns[i]*Math.hypot(wheelOffsets[i].x,wheelOffsets[i].z)*heading+deliveryDistance+roughDistance)/w.radius);
    }
    robot.updateMatrixWorld(true);
    let close=0;
    if(t>=24&&t<37)close=progress(t,24,26);
    else if(t>=37&&t<40)close=1-progress(t,37,39);
    else if(t>=41&&t<56)close=progress(t,41,43);
    else if(t>=56&&t<59)close=1-progress(t,56,59);
    else if(t>=62&&t<85)close=progress(t,62,64);
    else if(t>=85&&t<87)close=1-progress(t,85,87);
    k.grip(t<41?close:t<59?close:0,t<41?0:close);
    lid.rotation.z=t<47?-1.38: t<53?-1.38*(1-progress(t,47,53)):0;
    if(t<15){k.poseArm([0,0,0]);phase=t<3.5?'Raising the antenna':t<6?'Ready to drive':'Driving to the sample';}
    else if(t<19){const q=k.solveArm(ready,toolAngle);k.poseArm(q.map(v=>v*progress(t,15,19)));phase='Preparing the arm';}
    else if(t<22){placeArm(interpolate(ready,aboveTube,progress(t,19,22)));phase='Reaching for the test tube';}
    else if(t<24){placeArm(interpolate(aboveTube,groundTube,progress(t,22,24)));phase='Aligning the claws';}
    else if(t<26){placeArm(groundTube);phase='Grasping the test tube';}
    else if(t<29){placeArm(interpolate(groundTube,aboveTube,progress(t,26,29)));phase='Lifting the sample';}
    else if(t<33){placeArm(interpolate(aboveTube,aboveBox,progress(t,29,33)));phase='Moving the sample over the box';}
    else if(t<37){placeArm(interpolate(aboveBox,deposit,progress(t,33,37)));phase='Placing the sample inside the box';}
    else if(t<39){placeArm(deposit);phase='Releasing the sample';}
    else if(t<41){placeArm(interpolate(deposit,aboveBox,progress(t,39,41)));phase='Withdrawing the gripper';}
    else if(t<43){placeArm(aboveBox);phase='Closing the gripper';}
    else if(t<47){
      // Interpolate the same physical claw contact point to avoid a tool-frame jump.
      const q=k.solveArm(aboveBox,toolAngle);k.poseArm(q);const start=positionFor(k.pushLocal);
      const end=lidPoint(-1.38),highStart=start.clone().setY(.61),highEnd=end.clone().setY(.61);
      const target=t<44.3?interpolate(start,highStart,progress(t,43,44.3)):t<45.7?interpolate(highStart,highEnd,progress(t,44.3,45.7)):interpolate(highEnd,end,progress(t,45.7,47));
      placeArm(target,k.pushLocal);phase='Reaching the lid';
    }else if(t<53){placeArm(lidPoint(lid.rotation.z),k.pushLocal);phase='Closing the sample box';}
    else if(t<56){const start=lidPoint(0);placeArm(interpolate(start,start.clone().add(V(-.07,.15,0)),progress(t,53,56)),k.pushLocal);phase='Retracting from the box';}
    else if(t<59){const start=lidPoint(0).add(V(-.07,.15,0));const qStart=k.solveArm(start,toolAngle,k.pushLocal),qEnd=k.solveArm(aboveHandle,toolAngle,k.fingerToolLocal);const a=progress(t,56,59);k.poseArm(qStart.map((v,i)=>THREE.MathUtils.lerp(v,qEnd[i],a)));phase='Reaching for the box handle';}
    else if(t<62){placeArm(interpolate(aboveHandle,boxGrasp,progress(t,59,62)),k.fingerToolLocal);phase='Aligning with the box handle';}
    else if(t<64){placeArm(boxGrasp,k.fingerToolLocal);phase='Grasping the closed box';}
    else if(t<67){placeArm(interpolate(boxGrasp,carry,progress(t,64,67)),k.fingerToolLocal);phase='Lifting the box';}
    else if(t<69){placeArm(carry,k.fingerToolLocal);phase='Looking toward the delivery route';}
    else if(t<75){placeArm(carry,k.fingerToolLocal);phase='Turning 90 degrees';}
    else if(t<77){placeArm(carry,k.fingerToolLocal);phase='Straightening the wheels';}
    else if(t<81){placeArm(carry,k.fingerToolLocal);phase='Transporting the box';}
    else if(t<85){placeArm(interpolate(carry,boxGrasp,progress(t,81,85)),k.fingerToolLocal);phase='Setting the box down';}
    else if(t<87){placeArm(boxGrasp,k.fingerToolLocal);phase='Releasing the box';}
    else if(t<90){placeArm(interpolate(boxGrasp,aboveHandle,progress(t,87,90)),k.fingerToolLocal);phase='Withdrawing from the delivered box';}
    else if(t<94){const a=k.solveArm(aboveHandle,toolAngle,k.fingerToolLocal),b=k.solveArm(ready,toolAngle),u=progress(t,90,94);k.poseArm(a.map((v,i)=>THREE.MathUtils.lerp(v,b[i],u)));phase='Returning the arm';}
    else if(t<98){const q=k.solveArm(ready,toolAngle),u=1-progress(t,94,98);k.poseArm(q.map(v=>v*u));phase='Relaxing the arm';}
    else{k.poseArm([0,0,0]);phase=t<100?'Looking toward the terrain':t<106?'Turning toward the terrain':t<108?'Straightening the wheels':t<130?'Crossing uneven terrain':'Terrain crossing complete';}
    robot.updateMatrixWorld(true);
    // A rigid grasp keeps the tube fixed relative to the wrist for the whole carry.
    if(t>=26&&t<37){
      if(!attachment){
        const saved=[...['Arm Bicep Revolute','Arm Forearm Revolute','Diff Wrist Pitch'].map(n=>robot.joints[n].jointValue[0])];
        placeArm(groundTube);attachment=new THREE.Matrix4().copy(k.wrist.matrixWorld).invert().multiply(new THREE.Matrix4().makeTranslation(...groundTube.toArray()));k.poseArm(saved);
      }
      const carried=k.wrist.matrixWorld.clone().multiply(attachment);carried.decompose(tube.position,tube.quaternion,tube.scale);graspError=tube.position.distanceTo(k.toolPosition());
    }else{tube.position.copy(t<37?groundTube:deposit);tube.quaternion.identity();attachment=null;graspError=0;}
    // Apply the rover's rigid turn after solving the arm in its original planar frame.
    const yaw=new THREE.Quaternion().setFromAxisAngle(V(0,1,0),heading),advance=V(-roughDistance,0,-deliveryDistance);
    robot.position.sub(turnPivot).applyQuaternion(yaw).add(turnPivot).add(advance);
    robot.quaternion.copy(yaw).multiply(originalQuaternion);robot.updateMatrixWorld(true);
    if(t>=64&&t<85){
      k.wrist.matrixWorld.clone().multiply(boxAttachment).decompose(box.position,box.quaternion,box.scale);
    }else if(t>=85){
      const dropYaw=new THREE.Quaternion().setFromAxisAngle(V(0,1,0),Math.PI/2);
      box.position.copy(boxHome).sub(turnPivot).applyQuaternion(dropYaw).add(turnPivot).add(V(0,0,-.65));box.quaternion.copy(dropYaw);
    }
    if(t>=108)suspension.solve(bumpHeight);
    box.updateMatrixWorld(true);
    if(t>=37){tube.position.copy(box.localToWorld(V(0,deposit.y,0)));tube.quaternion.copy(box.quaternion);}
    tracks.forEach(mark=>{mark.visible=mark.position.x<1.36+drive&&drive>.01;});
    environment.updateMatrixWorld(true);
  }
  apply(0);
  return {
    description:'Mars sample collection and box delivery with synchronized grippers and closed finger linkages. The rover closes the box, lifts it by its handle, turns 90 degrees, drives 0.65 m, and sets it down. It then relaxes the arm, turns again, and traverses uneven terrain with a closed differential linkage. Kinematic demonstration.',
    viewScale:1.6,
    get phase(){return phase;},
    get cameraTarget(){const a=Math.PI/2*(progress(lastTime,69,75)+progress(lastTime,100,106));return V(2.15+progress(lastTime,6,15)*travel*.6,.55,toolZ).sub(turnPivot).applyAxisAngle(V(0,1,0),a).add(turnPivot).add(V(-4.7*progress(lastTime,108,130),0,-.65*progress(lastTime,77,81)));},
    update:apply,reset(){attachment=null;apply(0);},
    diagnostics(){return {time:lastTime,phase,closureError:k.closureError(),targetError,graspError,lidAngle:lid.rotation.z,tubePosition:tube.position.clone(),toolPosition:k.toolPosition(),pushPosition:positionFor(k.pushLocal),lidContact:lid.localToWorld(V(-boxWidth+.003,.009,contactZ)),claws:k.claws.map(c=>c.value),fingers:k.fingerLoops.map(f=>f.back.value),tower:robot.joints['Upper Tower Hinge'].jointValue[0],gimbal:robot.joints['Gimbal Dynamic Revolute'].jointValue[0],suspension:suspension.diagnostics(),roughDistance:4.7*progress(lastTime,108,130),heading:Math.PI/2*(progress(lastTime,69,75)+progress(lastTime,100,106)),deliveryDistance:.65*progress(lastTime,77,81),boxAttachment:boxAttachment.clone(),wheelAngles:k.wheels.map(w=>w.c.value),cameraDirection:V(0,0,-1).transformDirection(robot.links.Camera.matrixWorld),fingerPosition:positionFor(k.fingerToolLocal),handleBar,tube,box,lid,lidPanel,environment};},
    dispose(){scene.remove(environment);const geometries=new Set(),materials=new Set();environment.traverse(o=>{if(o.geometry)geometries.add(o.geometry);if(o.material)materials.add(o.material);});geometries.forEach(g=>g.dispose());materials.forEach(m=>m.dispose());scene.background=originalBackground;scene.fog=originalFog;robot.position.copy(originalPosition);robot.quaternion.copy(originalQuaternion);}
  };
}
