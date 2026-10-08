import * as THREE from 'three';
import {createV3Suspension} from './v3-suspension.js?v=20261008-animation2';
const V=(x=0,y=0,z=0)=>new THREE.Vector3(x,y,z);
const smooth=t=>t*t*t*(t*(t*6-15)+10);
export function createV3PayloadMotion(robot,scene){
 const suspension=createV3Suspension(robot,()=>0);
 const mount=robot.joints.YAM_payload_mount;
 if(!mount||mount.jointType!=='fixed')throw new Error('The YAM arm must be fixed to the V3 chassis lid.');
 const joints=Array.from({length:6},(_,i)=>robot.joints[`YAM_joint${i+1}`]);
 if(joints.some(j=>!j))throw new Error('The YAM arm is missing a required joint.');
 const tool=robot.links.YAM_link_6,gripLocal=V(0,-.033,.132);
 const floor=new THREE.Mesh(new THREE.CircleGeometry(1.35,80),new THREE.MeshStandardMaterial({color:0x242a30,roughness:.95}));
 floor.rotation.x=-Math.PI/2;floor.position.y=-.001;scene.add(floor);
 const geometry=new THREE.IcosahedronGeometry(.018,2),positions=geometry.attributes.position;
 for(let i=0;i<positions.count;i++){const p=V().fromBufferAttribute(positions,i);p.multiplyScalar(.91+.09*Math.sin(p.x*317+p.y*193+p.z*251));positions.setXYZ(i,p.x,p.y,p.z);}
 geometry.computeVertexNormals();geometry.computeBoundingBox();
 const rock=new THREE.Mesh(geometry,new THREE.MeshStandardMaterial({color:0x77736d,roughness:1}));rock.castShadow=true;rock.receiveShadow=true;scene.add(rock);
 const ground=V(-.35,-geometry.boundingBox.min.y,0);
 suspension.pose(0,0,0,0);
 const setAngles=angles=>{joints.forEach((j,i)=>j.setJointValue(angles[i]));tool.updateWorldMatrix(true,false);};
 const tip=()=>tool.localToWorld(gripLocal.clone());
 // Solve the original six-joint chain with its source limits. The gripper's
 // forward axis points down, so its fingers straddle the rock from above.
 function solve(target,seed){
  let angles=seed.slice();
  const residual=a=>{setAngles(a);const p=tip().sub(target),axis=V(0,0,1).transformDirection(tool.matrixWorld).sub(V(0,-1,0)).multiplyScalar(.12);return [...p.toArray(),...axis.toArray()];};
  for(let iteration=0;iteration<180;iteration++){
   const r=residual(angles);if(Math.max(...r.map(Math.abs))<.00001)break;
   const jac=angles.map((_,i)=>{const a=angles.slice();a[i]+=1e-5;return residual(a).map((v,k)=>(v-r[k])/1e-5);});
   const a=angles.map((_,i)=>[...angles.map((_,j)=>jac[i].reduce((s,v,k)=>s+v*jac[j][k],0)+(i===j?.00002:0)),-jac[i].reduce((s,v,k)=>s+v*r[k],0)]);
   for(let i=0;i<6;i++){let pivot=i;for(let j=i+1;j<6;j++)if(Math.abs(a[j][i])>Math.abs(a[pivot][i]))pivot=j;[a[i],a[pivot]]=[a[pivot],a[i]];const d=a[i][i];for(let k=i;k<=6;k++)a[i][k]/=d;for(let j=0;j<6;j++)if(j!==i){const f=a[j][i];for(let k=i;k<=6;k++)a[j][k]-=f*a[i][k];}}
   angles=angles.map((v,i)=>THREE.MathUtils.clamp(v+THREE.MathUtils.clamp(a[i][6],-.12,.12),joints[i].limit.lower,joints[i].limit.upper));
  }
  setAngles(angles);if(tip().distanceTo(target)>.002)throw new Error('The rock pickup target is outside the YAM arm workspace.');return angles;
 }
 const grasp=solve(ground,[2.9,2.4,1.5,0,.7,0]);
 const approach=solve(ground.clone().add(V(0,.16,0)),grasp);
 const lift=solve(ground.clone().add(V(0,.38,0)),approach);
 const home=solve(ground.clone().add(V(.04,.38,0)),lift);
 // Transfer a quarter-turn from the front to the side within base limits.
 // Lift clear of the chassis before this sweep.
 const turn=angles=>{const result=angles.slice();result[0]-=Math.PI/2;if(result[0]>joints[0].limit.upper)throw new Error('YAM transfer exceeds the base joint limit.');return result;};
 const destinationGrasp=turn(grasp),destinationLift=turn(lift),destinationHome=turn(home),destinationApproach=turn(approach);
 setAngles(destinationGrasp);const destination=tip();
 setAngles(grasp);const rockGripRotation=tool.getWorldQuaternion(new THREE.Quaternion()).invert();
 // Negative prismatic travel opens this exported gripper. Match the closing
 // distance to the actual rock extent along the fingers' local closing axis.
 let rockHalfWidth=0;
 for(let i=0;i<positions.count;i++)rockHalfWidth=Math.max(rockHalfWidth,Math.abs(V().fromBufferAttribute(positions,i).applyQuaternion(rockGripRotation).x));
 const openGap=-.035,closedGap=-rockHalfWidth;
 let elapsed=0,phase='',attached=false,graspError=0;
 function pose(){
  const t=elapsed%30,returning=Math.floor(elapsed/30)%2===1;
  const source=returning?destination:ground,placed=returning?ground:destination;
  const sourceGrasp=returning?destinationGrasp:grasp,sourceApproach=returning?destinationApproach:approach;
  const sourceLift=returning?destinationLift:lift,targetLift=returning?lift:destinationLift,targetGrasp=returning?grasp:destinationGrasp;
  const sourceHome=returning?destinationHome:home,targetHome=returning?home:destinationHome;
  let from=sourceHome,to=sourceHome,u=0,gap=openGap;
  if(t<2){phase='Ready to collect';}
  else if(t<6){phase='Reaching toward rock';from=sourceHome;to=sourceApproach;u=(t-2)/4;}
  else if(t<9){phase='Lowering gripper';from=sourceApproach;to=sourceGrasp;u=(t-6)/3;}
  else if(t<10.5){phase='Gripping rock';from=to=sourceGrasp;gap=THREE.MathUtils.lerp(openGap,closedGap,smooth((t-9)/1.5));}
  else if(t<14.5){phase='Lifting rock';from=sourceGrasp;to=sourceLift;u=(t-10.5)/4;gap=closedGap;}
  else if(t<20.5){phase=returning?'Returning rock to front':'Swiveling toward side';from=sourceLift;to=targetLift;u=(t-14.5)/6;gap=closedGap;}
  else if(t<24){phase='Placing rock on side';from=targetLift;to=targetGrasp;u=(t-20.5)/3.5;gap=closedGap;}
  else if(t<25.5){phase='Releasing rock';from=to=targetGrasp;gap=THREE.MathUtils.lerp(closedGap,openGap,smooth((t-24)/1.5));}
  else{phase='Retracting arm';from=targetGrasp;to=targetHome;u=(t-25.5)/4.5;}
  setAngles(from.map((v,i)=>THREE.MathUtils.lerp(v,to[i],smooth(u))));
  robot.joints.YAM_right_finger.setJointValue(gap);
  attached=t>=10.5&&t<24;
  if(attached){rock.position.copy(tip());rock.quaternion.copy(tool.getWorldQuaternion(new THREE.Quaternion())).multiply(rockGripRotation);}
  else{
   rock.position.copy(t>=24?placed:source);
   const onDestination=returning?t<24:t>=24;
   setAngles(onDestination?destinationGrasp:grasp);rock.quaternion.copy(tool.getWorldQuaternion(new THREE.Quaternion())).multiply(rockGripRotation);
   setAngles(from.map((v,i)=>THREE.MathUtils.lerp(v,to[i],smooth(u))));
  }
  if(t>=9&&t<10.5)graspError=tip().distanceTo(source);
  robot.updateMatrixWorld(true);
 }
 pose();
 return {
  description:'The chassis-mounted YAM arm reaches for a rock in front of V3, closes its gripper, lifts the rock, swivels to place it beside the rover, then returns it on the next pass.',
  get phase(){return phase;},
  update(time,delta){elapsed+=delta;pose();},
  reset(){elapsed=0;graspError=0;pose();},
  diagnostics(){return {...suspension.diagnostics(),armAngles:joints.map(j=>j.jointValue[0]),mountLocal:mount.position.toArray(),mountWorld:mount.getWorldPosition(V()).toArray(),phase,attached,graspError,gripLocal:gripLocal.toArray(),fingerHalfGap:-robot.joints.YAM_right_finger.jointValue[0],rockHalfWidth,rockPosition:rock.position.toArray(),gripPosition:tip().toArray(),groundPosition:ground.toArray(),destinationPosition:destination.toArray()};},
  dispose(){for(const mesh of [floor,rock]){scene.remove(mesh);mesh.geometry.dispose();mesh.material.dispose();}}
 };
}












