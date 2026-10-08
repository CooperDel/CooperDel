import * as THREE from 'three';
import { createUUVManipulator } from './uuv-manipulator.js';
import { createInspectionCourse } from './uuv-inspection-course.js';
import { createUUVThrusters } from './uuv-thrusters.js';

// A visual swim demonstration: coordinated thrust vectoring, not fluid dynamics.
export function createUUVMotion(robot, scene) {
  const manipulator = createUUVManipulator(robot);
  const originalPosition = robot.position.clone(), originalQuaternion = robot.quaternion.clone();
  robot.updateMatrixWorld(true);
  const bounds = new THREE.Box3().setFromObject(robot);
  const center = bounds.getCenter(new THREE.Vector3());
  const size = Math.max(...bounds.getSize(new THREE.Vector3()).toArray());
  const centerLocal = robot.worldToLocal(center.clone());
  const propulsion=createUUVThrusters(robot,centerLocal,size);
  const thrusters=propulsion.entries;
  const environment = new THREE.Group(); environment.name = 'UUV underwater environment'; scene.add(environment);
  const oldBackground = scene.background, oldFog = scene.fog;
  scene.background = new THREE.Color(0x073b4b);
  scene.fog = new THREE.FogExp2(0x073b4b, .075 / size);
  const floorY = bounds.min.y - size * .85;
  let seed = 7183;
  const random = () => { seed = (Math.imul(seed,1664525)+1013904223) >>> 0; return seed / 4294967296; };
  const sandHeight = (x,z) => floorY + size * (.028 * Math.sin(x / size * 4) * Math.cos(z / size * 3) + .012 * Math.sin(x / size * 27 + z / size * 9));
  const floorGeometry = new THREE.PlaneGeometry(size*24,size*24,120,120); floorGeometry.rotateX(-Math.PI/2);
  const points = floorGeometry.attributes.position, colors = [];
  for (let i=0;i<points.count;i++) {
    const x = points.getX(i)+center.x, z = points.getZ(i)+center.z;
    points.setXYZ(i,x,sandHeight(x,z),z);
    const glow = .8 + .15 * Math.sin(x/size*17 + Math.cos(z/size*13));
    colors.push(.38*glow,.43*glow,.32*glow);
  }
  floorGeometry.setAttribute('color',new THREE.Float32BufferAttribute(colors,3)); floorGeometry.computeVertexNormals();
  environment.add(new THREE.Mesh(floorGeometry,new THREE.MeshStandardMaterial({vertexColors:true,roughness:1})));
  const rockGeometry = new THREE.DodecahedronGeometry(1,1), rockMaterial = new THREE.MeshStandardMaterial({color:0x536b64,roughness:1,flatShading:true});
  const rocks = new THREE.InstancedMesh(rockGeometry,rockMaterial,65), dummy = new THREE.Object3D();
  for (let i=0;i<65;i++) {
    const x=center.x+(random()-.5)*size*17, z=center.z+(random()-.5)*size*17, scale=size*(.05+random()*.17);
    dummy.position.set(x,sandHeight(x,z)-scale*.15,z);dummy.rotation.set(random(),random()*6,random());dummy.scale.set(scale*1.5,scale*.65,scale);dummy.updateMatrix();rocks.setMatrixAt(i,dummy.matrix);
  }
  environment.add(rocks);
  const grassGeometry = new THREE.ConeGeometry(size*.022,size*.35,4), grassMaterial = new THREE.MeshStandardMaterial({color:0x286b58,roughness:1});
  const grass = new THREE.InstancedMesh(grassGeometry,grassMaterial,220);
  for (let i=0;i<220;i++) {
    const x=center.x+(random()-.5)*size*15,z=center.z+(random()-.5)*size*15,h=.5+random();
    dummy.position.set(x,sandHeight(x,z)+size*.175*h,z);dummy.rotation.set((random()-.5)*.4,random()*6,(random()-.5)*.4);dummy.scale.set(1,h,1);dummy.updateMatrix();grass.setMatrixAt(i,dummy.matrix);
  }
  environment.add(grass);
  const shaftMaterial = new THREE.MeshBasicMaterial({color:0x8fdddf,transparent:true,opacity:.035,depthWrite:false,blending:THREE.AdditiveBlending,side:THREE.DoubleSide});
  const shaftGeometry = new THREE.CylinderGeometry(size*.05,size*.48,size*6,12,1,true);
  for (let i=0;i<7;i++) {
    const shaft = new THREE.Mesh(shaftGeometry,shaftMaterial);shaft.position.set(center.x+(i-3)*size*1.4,center.y+size*1.7,center.z-size*2);shaft.rotation.z=-.22;environment.add(shaft);
  }
  const dustPositions = new Float32Array(600*3);
  for(let i=0;i<600;i++) dustPositions.set([center.x+(random()-.5)*size*14,floorY+random()*size*6,center.z+(random()-.5)*size*14],i*3);
  const dustGeometry = new THREE.BufferGeometry();dustGeometry.setAttribute('position',new THREE.BufferAttribute(dustPositions,3));
  environment.add(new THREE.Points(dustGeometry,new THREE.PointsMaterial({color:0xc2e5df,size:size*.006,transparent:true,opacity:.4,depthWrite:false})));
  // Short streams show thrust without spinning the entire solid thruster housing.
  const wakes = thrusters.map(() => {
    const geometry = new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(new Float32Array(40*3),3));
    const mesh = new THREE.Points(geometry,new THREE.PointsMaterial({color:0xbde8ee,size:size*.009,transparent:true,opacity:.55,depthWrite:false}));environment.add(mesh);return mesh;
  });
  const course=createInspectionCourse({center,centerLocal,homeTip:manipulator.homePosition,baseQuaternion:originalQuaternion,size,environment,floorY});
  const target=center.clone();
  let elapsed=0,phase='',inspectionState;
  const angularDelta=(a,b)=>{
    const q=b.clone().multiply(a.clone().invert());if(q.w<0)q.set(-q.x,-q.y,-q.z,-q.w);
    const length=Math.hypot(q.x,q.y,q.z);return length<1e-10?new THREE.Vector3():new THREE.Vector3(q.x,q.y,q.z).multiplyScalar(2*Math.atan2(length,q.w)/length);
  };
  function pose(t,dt=.016) {
    const state=course.sample(t),h=.025,before=course.sample(t-h),after=course.sample(t+h);
    inspectionState=state;phase=state.phase;
    if(!manipulator.moveTo(state.tip))throw new Error('Inspection path exceeds the closed-loop manipulator workspace.');
    robot.quaternion.copy(state.quaternion);
    robot.position.copy(state.position).sub(centerLocal.clone().multiply(robot.scale).applyQuaternion(robot.quaternion));
    const velocity=after.position.clone().sub(before.position).multiplyScalar(1/(2*h));
    const acceleration=after.position.clone().add(before.position).addScaledVector(state.position,-2).multiplyScalar(1/(h*h));
    const omega=angularDelta(before.quaternion,after.quaternion).multiplyScalar(1/(2*h));
    const alpha=angularDelta(state.quaternion,after.quaternion).sub(angularDelta(before.quaternion,state.quaternion)).multiplyScalar(1/(h*h));
    const inverse=robot.quaternion.clone().invert();
    const force=velocity.multiplyScalar(.65/size).addScaledVector(acceleration,.25/size).add(new THREE.Vector3(.018,0,0)).applyQuaternion(inverse);
    const torque=omega.multiplyScalar(.15).addScaledVector(alpha,.05).applyQuaternion(inverse);
    propulsion.update(force,torque,dt);
    robot.updateMatrixWorld(true);
    const actualTip=robot.links.frame_ee_tip.getWorldPosition(new THREE.Vector3());
    target.copy(state.position).lerp(actualTip,.18);
    course.updateProgress(state);
    wakes.forEach((wake,i)=>{
      const thruster=thrusters[i],strength=Math.abs(thruster.throttle);
      wake.visible=strength>.015;
      wake.material.opacity=.2+.6*strength;
      const origin=thruster.joint.getWorldPosition(new THREE.Vector3());
      const direction=thruster.direction.clone().applyQuaternion(robot.quaternion).multiplyScalar(thruster.throttle>=0?-1:1);
      const positions=wake.geometry.attributes.position;
      for(let k=0;k<40;k++){
        const progress=((k/40+t*(.5+strength*3))%1),spread=size*.015*progress;
        const p=origin.clone().addScaledVector(direction,size*(.06+.4*strength)*progress);
        p.x+=Math.sin(k*13.7)*spread;p.y+=Math.cos(k*7.3)*spread;
        positions.setXYZ(k,p.x,p.y,p.z);
      }
      positions.needsUpdate=true;wake.geometry.computeBoundingSphere();
    });
  }
  pose(0);
  return {
    viewScale:1.8,
    get cameraTarget(){return target;},
    get phase(){return phase;},
    get inspection(){return inspectionState;},
    get duration(){return course.duration;},
    get thrusterStates(){return thrusters.map(e=>({angle:e.joint.jointValue[0],throttle:e.throttle,direction:e.direction.clone()}));},
    description:'First the UUV locates and traces a pipe crack, then approaches a slanted wall, pitches upward and traces its crack. The tool follows both surfaces with closed-loop linkage constraints. Thruster direction and flow respond to acceleration, turning, braking and station keeping. This is an illustrative kinematic inspection, not a fluid or contact simulation.',
    update(_time,dt){elapsed+=dt;pose(elapsed,dt);},
    reset(){elapsed=0;manipulator.reset();propulsion.reset();pose(0);},
    get manipulator(){return manipulator;},
    dispose(){
      manipulator.dispose();
      scene.remove(environment);scene.background=oldBackground;scene.fog=oldFog;
      const geometries=new Set(),materials=new Set();
      environment.traverse(object=>{if(object.geometry)geometries.add(object.geometry);if(object.material)materials.add(object.material);});
      geometries.forEach(g=>g.dispose());materials.forEach(m=>m.dispose());
      propulsion.dispose();
      robot.position.copy(originalPosition);robot.quaternion.copy(originalQuaternion);
    }
  };
}
