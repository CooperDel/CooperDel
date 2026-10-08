import * as THREE from 'three';
import { coursePose, courseLength, rocks, block, rockHeight, rockSurface, lunarHeight } from './nova-course.js?v=20261008-animation2';

// Kinematic closed linkage. Connector ends are idealized spherical bearings.
export function createNovaMotion(robot, scene) {
  const V=(x=0,y=0,z=0)=>new THREE.Vector3(x,y,z);
  const joint=name=>{const j=robot.joints[`${name} Revolute`];if(!j)throw new Error(`Missing NOVA joint: ${name}`);return j;};
  const localBounds=link=>{
    link.updateWorldMatrix(true,true);
    const inverse=link.matrixWorld.clone().invert(),box=new THREE.Box3();
    link.traverse(child=>{if(child.isMesh){child.geometry.computeBoundingBox();box.union(child.geometry.boundingBox.clone().applyMatrix4(new THREE.Matrix4().multiplyMatrices(inverse,child.matrixWorld)));}});
    return box;
  };
  const basePosition=robot.position.clone(),baseQuaternion=robot.quaternion.clone();
  robot.updateMatrixWorld(true);
  const modules=['Front Left','Back Left','Front Right','Back Right'].map(name=>{
    const wheel=joint(`${name} Wheel`),steering=joint(`${name} Swerve`),link=robot.links[`${name} Wheel`];
    const bounds=localBounds(link),center=bounds.getCenter(V());
    // Rear-left export has both a translated STL and a joint at the front axle.
    // Recenter its mesh and put its pivot on the matching rear swerve shaft.
    if(name.startsWith('Back'))wheel.position.copy(joint(`${name.replace('Back','Front')} Wheel`).position);
    if(wheel.origPosition)wheel.origPosition.copy(wheel.position);
    link.position.add(V(0,-center.y,-center.z));
    steering.limit.lower=-Math.PI/2;steering.limit.upper=Math.PI/2;
    return {name,wheel,steering,link,center,radius:(bounds.max.y-bounds.min.y)/2,phase:0};
  });
  const rockers=[joint('Left Drivetrain'),joint('Right Drivetrain')],bar=joint('Diff Bar');
  [...rockers,bar].forEach(j=>{j.limit.lower=-.45;j.limit.upper=.45;j.setJointValue(0);});
  robot.updateMatrixWorld(true);
  const worldCenter=m=>m.link.localToWorld(m.center.clone());
  const wheelCenters=modules.map(worldCenter);
  const forward=wheelCenters[0].clone().sub(wheelCenters[1]);forward.y=0;forward.normalize();
  const up=V(0,1,0),side=up.clone().cross(forward).normalize();
  const reference=rockers[0].getWorldPosition(V()).add(rockers[1].getWorldPosition(V())).multiplyScalar(.5);
  const localReference=robot.worldToLocal(reference.clone());
  const ground=wheelCenters.reduce((sum,c,i)=>sum+c.y-modules[i].radius,0)/4;
  const connectors=['Left','Right'].map(name=>{
    const j=joint(`${name} Diff Bar Connection`),link=robot.links[`${name} Diff Bar Connection`];
    const c=localBounds(link).getCenter(V()),start=V(c.x,0,0),end=V(c.x,2*c.y,2*c.z);
    const anchor=bar.worldToLocal(link.localToWorld(end.clone()));
    return {j,link,start,end,anchor,length:start.distanceTo(end)};
  });
  for(const m of modules){
    m.steerSign=Math.sign(m.steering.axis.clone().transformDirection(m.steering.matrixWorld).dot(up));
    m.spinSign=Math.sign(m.wheel.axis.clone().transformDirection(m.wheel.matrixWorld).cross(up).dot(forward));
    const offset=m.steering.getWorldPosition(V()).sub(reference);
    m.forwardOffset=offset.dot(forward);m.sideOffset=offset.dot(side);
  }
  let distance=0,lateral=0,heading=0,elapsed=0,phase='Crossing rocks',state=[0,0,0,0,0,0];
  const terrainHeight=(x,z)=>{
    const p=V(x-reference.x,0,z-reference.z),f=p.dot(forward)+distance,s=p.dot(side)+lateral;
    return ground+lunarHeight(f,s)+rockHeight(f,s);
  };
  const geometry=new THREE.PlaneGeometry(2.2,2.2,64,64);geometry.rotateX(-Math.PI/2);
  geometry.setAttribute('color',new THREE.BufferAttribute(new Float32Array(geometry.attributes.position.count*3),3));
  const terrain=new THREE.Mesh(geometry,new THREE.MeshStandardMaterial({vertexColors:true,roughness:1,metalness:0,side:THREE.DoubleSide}));
  terrain.position.set(reference.x,0,reference.z);scene.add(terrain);
  const obstacles=[];
  const rockMaterial=()=>new THREE.MeshStandardMaterial({vertexColors:true,roughness:1,metalness:0,flatShading:true,side:THREE.DoubleSide});
  function shadeRock(geo){
    const positions=geo.attributes.position,colors=new Float32Array(positions.count*3);
    for(let i=0;i<positions.count;i++){
      const x=positions.getX(i),y=positions.getY(i),z=positions.getZ(i);
      const grain=Math.sin(x*287+y*173+z*229)*Math.cos(x*137-z*317);
      const veins=Math.pow(Math.abs(Math.sin(x*63+y*45-z*31)),14);
      const value=.19+.045*grain-.055*veins;
      colors.set([value,value*.99,value*.96],i*3);
    }
    geo.setAttribute('color',new THREE.BufferAttribute(colors,3));geo.computeVertexNormals();
  }
  for(let cycle=-1;cycle<=1;cycle++){
    for(const rock of rocks){
      const geo=new THREE.PlaneGeometry(rock.r*2.5,rock.r*2.5,24,24);geo.rotateX(-Math.PI/2);
      const p=geo.attributes.position;
      for(let i=0;i<p.count;i++)p.setY(i,rockSurface(rock,p.getZ(i),p.getX(i)));
      // Remove flat square skirts so only the irregular rock silhouette remains.
      const indices=geo.index.array,kept=[];
      for(let i=0;i<indices.length;i+=3)if([indices[i],indices[i+1],indices[i+2]].some(v=>p.getY(v)>.00005))kept.push(indices[i],indices[i+1],indices[i+2]);
      geo.setIndex(kept);shadeRock(geo);
      const mesh=new THREE.Mesh(geo,rockMaterial());
      mesh.quaternion.setFromUnitVectors(V(0,0,1),forward);
      scene.add(mesh);obstacles.push({mesh,cycle,f:rock.f,s:rock.s});
    }
    const boulder=new THREE.IcosahedronGeometry(1,2),bp=boulder.attributes.position;
    for(let i=0;i<bp.count;i++){
      const x=bp.getX(i),y=bp.getY(i),z=bp.getZ(i);
      const scale=1+.13*Math.sin(x*8+y*5-z*6)+.08*Math.sin(z*13-x*9);
      bp.setXYZ(i,x*scale,y*scale,z*scale);
    }
    boulder.computeBoundingBox();
    const center=boulder.boundingBox.getCenter(V()),size=boulder.boundingBox.getSize(V());
    boulder.translate(-center.x,-center.y,-center.z);
    boulder.scale(block.width/size.x,block.height/size.y,block.length/size.z);
    shadeRock(boulder);
    const mesh=new THREE.Mesh(boulder,rockMaterial());
    mesh.quaternion.setFromUnitVectors(V(0,0,1),forward);scene.add(mesh);
    obstacles.push({mesh,cycle,f:block.f,s:block.s,block:true});
  }
  function apply(values){
    const [height,pitch,roll,left,right,diff]=values;
    robot.quaternion.copy(new THREE.Quaternion().setFromAxisAngle(up,heading)).multiply(new THREE.Quaternion().setFromAxisAngle(side,pitch)).multiply(new THREE.Quaternion().setFromAxisAngle(forward,roll)).multiply(baseQuaternion);
    robot.position.copy(reference).sub(localReference.clone().applyQuaternion(robot.quaternion));robot.position.y+=height;
    rockers[0].setJointValue(left);rockers[1].setJointValue(right);bar.setJointValue(diff);robot.updateMatrixWorld(true);
  }
  function residual(values){
    apply(values);
    const errors=modules.map(m=>{const p=worldCenter(m);return p.y-m.radius-terrainHeight(p.x,p.z);});
    for(const c of connectors){const a=c.j.localToWorld(c.start.clone()),b=bar.localToWorld(c.anchor.clone());errors.push(a.distanceTo(b)-c.length);}
    return errors;
  }
  function solve(){
    for(let iteration=0;iteration<10;iteration++){
      const r=residual(state);if(Math.max(...r.map(Math.abs))<.00003)break;
      const jac=state.map((_,i)=>{const s=state.slice();s[i]+=.0001;const v=residual(s);return v.map((n,k)=>(n-r[k])/.0001);});
      const a=state.map((_,i)=>[...state.map((_,j)=>jac[i].reduce((sum,n,k)=>sum+n*jac[j][k],0)+(i===j?1e-7:0)),-jac[i].reduce((sum,n,k)=>sum+n*r[k],0)]);
      for(let i=0;i<6;i++){
        let pivot=i;for(let j=i+1;j<6;j++)if(Math.abs(a[j][i])>Math.abs(a[pivot][i]))pivot=j;
        [a[i],a[pivot]]=[a[pivot],a[i]];const d=a[i][i];for(let k=i;k<=6;k++)a[i][k]/=d;
        for(let j=0;j<6;j++)if(j!==i){const f=a[j][i];for(let k=i;k<=6;k++)a[j][k]-=f*a[i][k];}
      }
      state=state.map((n,i)=>THREE.MathUtils.clamp(n+THREE.MathUtils.clamp(a[i][6],-.06,.06),i===0?-.12:-.4,i===0?.12:.4));
    }
    apply(state);
    for(const c of connectors){
      const target=c.j.worldToLocal(bar.localToWorld(c.anchor.clone())).sub(c.start);
      c.link.quaternion.setFromUnitVectors(c.end.clone().sub(c.start).normalize(),target.normalize());
      c.link.position.copy(c.start).sub(c.start.clone().applyQuaternion(c.link.quaternion));
    }
    robot.updateMatrixWorld(true);
  }
  function drawTerrain(){
    const p=geometry.attributes.position,colors=geometry.attributes.color;
    for(let i=0;i<p.count;i++){
      const offset=V(p.getX(i),0,p.getZ(i)),f=offset.dot(forward)+distance,s=offset.dot(side)+lateral;
      const h=lunarHeight(f,s),grain=Math.sin(f*113+s*71)*Math.sin(f*59-s*127);
      const shade=THREE.MathUtils.clamp(.22+.025*grain+.018*Math.sin(f*17+s*23)+h*.8,.12,.32);
      p.setY(i,ground+h-.001);colors.setXYZ(i,shade,shade,shade*.97);
    }
    colors.needsUpdate=true;
    p.needsUpdate=true;geometry.computeVertexNormals();geometry.computeBoundingSphere();
    const cycle=Math.floor(distance/courseLength);
    for(const o of obstacles){
      o.mesh.position.copy(reference).addScaledVector(forward,o.f+(cycle+o.cycle)*courseLength-distance).addScaledVector(side,o.s-lateral);
      o.mesh.position.y=ground+lunarHeight(o.f,o.s)+(o.block?block.height/2:0);
      o.mesh.visible=Math.abs(o.f+(cycle+o.cycle)*courseLength-distance)<1.3;
    }
  }
  function reset(){distance=lateral=heading=elapsed=0;phase='Crossing rocks';state=[0,0,0,0,0,0];modules.forEach(m=>{m.phase=0;m.speed=0;m.wheel.setJointValue(0);m.steering.setJointValue(0);});solve();drawTerrain();}
  reset();
  return {
    reset,
    get phase(){return phase;},
    update(time,delta){
      elapsed+=delta;
      const pose=coursePose(elapsed);distance=pose.distance;lateral=pose.lateral;phase=pose.label;
      modules.forEach(m=>{
        m.speed=pose.speed;
        m.steering.setJointValue(pose.angle*m.steerSign);
        m.phase+=m.speed/m.radius*delta*m.spinSign;m.wheel.setJointValue(m.phase);
      });solve();drawTerrain();
    },
    diagnostics(){return {contactErrors:residual(state).slice(0,4),closureErrors:connectors.map(c=>c.link.localToWorld(c.end.clone()).distanceTo(bar.localToWorld(c.anchor.clone()))),state:state.slice(),heading,distance,lateral,phase,steering:modules.map(m=>m.steering.jointValue[0]/m.steerSign),wheelSpeeds:modules.map(m=>m.speed)};},
    dispose(){scene.remove(terrain);geometry.dispose();terrain.material.dispose();for(const o of obstacles){scene.remove(o.mesh);o.mesh.geometry.dispose();o.mesh.material.dispose();}robot.position.copy(basePosition);robot.quaternion.copy(baseQuaternion);}
  };
}
