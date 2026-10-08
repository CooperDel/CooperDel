import * as THREE from 'three';
const smooth=t=>{t=THREE.MathUtils.clamp(t,0,1);return t*t*t*(t*(t*6-15)+10);};
const V=(x=0,y=0,z=0)=>new THREE.Vector3(x,y,z);

export function createInspectionCourse({center,centerLocal,homeTip,baseQuaternion,size,environment,floorY}) {
  const attitude=(yaw,pitch=0)=>new THREE.Quaternion().setFromAxisAngle(V(0,1,0),yaw).multiply(new THREE.Quaternion().setFromAxisAngle(V(1,0,0),pitch)).multiply(baseQuaternion);
  const tipAt=(position,quaternion,tip=homeTip)=>tip.clone().sub(centerLocal).applyQuaternion(quaternion).add(position);
  const localAt=(point,position,quaternion)=>point.clone().sub(position).applyQuaternion(quaternion.clone().invert()).add(centerLocal);
  const pipePose={position:center.clone().add(V(0,0,-size*1.3)),quaternion:attitude(0)};
  const wallPose={position:center.clone().add(V(size*2.5,size*.25,-size*1.8)),quaternion:attitude(-.95,.48)};
  const pipeContact=tipAt(pipePose.position,pipePose.quaternion),wallContact=tipAt(wallPose.position,wallPose.quaternion);
  const pipeRadius=size*.19,pipeCenter=pipeContact.clone().add(V(0,0,-pipeRadius));
  const wallRotation=new THREE.Quaternion().setFromAxisAngle(V(0,1,0),-.95).multiply(new THREE.Quaternion().setFromAxisAngle(V(1,0,0),.48));
  function crack(which,s){
    const x=(s-.5)*.034,y=.004*Math.sin(s*Math.PI*4)+.0012*Math.sin(s*Math.PI*10);
    if(which==='pipe')return V(pipeCenter.x+x,pipeCenter.y+y,pipeCenter.z+Math.sqrt(pipeRadius*pipeRadius-y*y));
    return V(x,y*.95,0).applyQuaternion(wallRotation).add(wallContact);
  }
  const pipeMaterial=new THREE.MeshStandardMaterial({color:0x658883,roughness:.65,metalness:.35});
  const pipe=new THREE.Mesh(new THREE.CylinderGeometry(pipeRadius,pipeRadius,size*2.1,48),pipeMaterial);pipe.rotation.z=Math.PI/2;pipe.position.copy(pipeCenter);environment.add(pipe);
  for(const side of [-1,1]){
    const flange=new THREE.Mesh(new THREE.CylinderGeometry(pipeRadius*1.23,pipeRadius*1.23,size*.07,48),new THREE.MeshStandardMaterial({color:0x7e6650,roughness:.85,metalness:.2}));flange.rotation.z=Math.PI/2;flange.position.copy(pipeCenter).add(V(side*size*.78,0,0));environment.add(flange);
    const height=pipeCenter.y-floorY;
    const support=new THREE.Mesh(new THREE.BoxGeometry(size*.12,height,size*.25),new THREE.MeshStandardMaterial({color:0x56625c,roughness:1}));support.position.set(pipeCenter.x+side*size*.65,floorY+height/2,pipeCenter.z);environment.add(support);
  }
  const wallDepth=size*.1;
  const wall=new THREE.Mesh(new THREE.BoxGeometry(size*1.65,size*2.8,wallDepth),new THREE.MeshStandardMaterial({color:0x84938a,roughness:.95,metalness:0}));wall.quaternion.copy(wallRotation);wall.position.copy(wallContact).add(V(0,0,-wallDepth/2).applyQuaternion(wallRotation));environment.add(wall);
  // Dark crack tubes sit just above each surface. A narrow gold trace shows progress.
  const traces={};
  for(const which of ['pipe','wall']){
    const points=Array.from({length:121},(_,i)=>crack(which,i/120));
    const curve=new THREE.CatmullRomCurve3(points);
    const line=new THREE.Mesh(new THREE.TubeGeometry(curve,120,.00075,5,false),new THREE.MeshStandardMaterial({color:0x111915,roughness:1}));line.name=`${which} crack`;environment.add(line);
    const progress=new THREE.BufferGeometry().setFromPoints(points);
    progress.setDrawRange(0,0);
    const trace=new THREE.Line(progress,new THREE.LineBasicMaterial({color:0xB3A369,depthTest:false,transparent:true,opacity:.95}));trace.renderOrder=3;environment.add(trace);traces[which]=progress;
  }
  const start={position:center.clone(),quaternion:attitude(-.65),tip:homeTip.clone()};
  const pipeStart=localAt(crack('pipe',0),pipePose.position,pipePose.quaternion),pipeEnd=localAt(crack('pipe',1),pipePose.position,pipePose.quaternion);
  const wallStart=localAt(crack('wall',0),wallPose.position,wallPose.quaternion),wallEnd=localAt(crack('wall',1),wallPose.position,wallPose.quaternion);
  const pipeDock={...pipePose,tip:pipeStart},pipeFinish={...pipePose,tip:pipeEnd};
  const retreat={position:pipePose.position.clone().add(V(0,0,size*.5)),quaternion:attitude(0),tip:homeTip.clone()};
  const wallBack=V(0,0,size*1.1).applyAxisAngle(V(0,1,0),-.95);
  const wallStage={position:wallPose.position.clone().add(wallBack),quaternion:attitude(-.35),tip:homeTip.clone()};
  const wallReady={...wallStage,quaternion:attitude(-.95)};
  const wallNear={position:wallPose.position.clone().add(V(0,0,size*.17).applyAxisAngle(V(0,1,0),-.95)),quaternion:attitude(-.95),tip:homeTip.clone()};
  const wallDock={...wallPose,tip:wallStart},wallFinish={...wallPose,tip:wallEnd};
  const segments=[];let end=0;
  function add(duration,label,from,to,inspection=null){segments.push({start:end,end:end+duration,label,from,to,inspection});end+=duration;}
  const pipeFacing={...start,quaternion:attitude(0)};
  add(4,'1 / Pipe · searching and turning',start,pipeFacing);
  add(7,'1 / Pipe · approach and decelerate',pipeFacing,pipeDock);
  add(2,'1 / Pipe · hold position and align tip',pipeDock,pipeDock);
  add(10,'1 / Pipe · trace crack',pipeDock,pipeFinish,'pipe');
  add(3,'1 / Pipe · withdraw from pipe',pipeFinish,retreat);
  add(5,'2 / Slanted wall · reposition',retreat,wallStage);
  add(4,'2 / Slanted wall · searching and turning',wallStage,wallReady);
  add(7,'2 / Slanted wall · approach and decelerate',wallReady,wallNear);
  add(4,'2 / Slanted wall · pitch up and align tip',wallNear,wallDock);
  add(10,'2 / Slanted wall · trace crack',wallDock,wallFinish,'wall');
  add(4,'2 / Slanted wall · withdraw and level',wallFinish,wallReady);
  add(6,'Return to starting water position',wallReady,start);
  function sample(time){
    const t=((time%end)+end)%end,segment=segments.find(s=>t<s.end)||segments[0],u=smooth((t-segment.start)/(segment.end-segment.start));
    const position=segment.from.position.clone().lerp(segment.to.position,u),quaternion=segment.from.quaternion.clone().slerp(segment.to.quaternion,u);
    let tip=segment.from.tip.clone().lerp(segment.to.tip,u),contact=null;
    if(segment.inspection){contact=crack(segment.inspection,u);tip=localAt(contact,position,quaternion);}
    return {position,quaternion,tip,contact,inspection:segment.inspection,progress:u,phase:segment.label,time:t};
  }
  return {sample,duration:end,segments,crack,
    updateProgress(state){
      for(const which of ['pipe','wall']){
        const traceSegment=segments.find(s=>s.inspection===which);
        const n=state.time<traceSegment.start?0:state.time>=traceSegment.end?121:Math.floor(state.progress*120)+1;
        traces[which].setDrawRange(0,n);
      }
    }
  };
}
