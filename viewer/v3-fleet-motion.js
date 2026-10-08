import * as THREE from 'three';
import {createV3Suspension} from './v3-suspension.js';
export const lanes=[-1.4,0,1.4];
let seed=83147;
const random=()=>{seed=(1664525*seed+1013904223)>>>0;return seed/4294967296;};
const hash=(x,z)=>{const n=Math.sin(x*127.1+z*311.7)*43758.5453;return n-Math.floor(n);};
const smooth=t=>t*t*(3-2*t);
function noise(x,z){
 const ix=Math.floor(x),iz=Math.floor(z),u=smooth(x-ix),v=smooth(z-iz);
 return THREE.MathUtils.lerp(THREE.MathUtils.lerp(hash(ix,iz),hash(ix+1,iz),u),THREE.MathUtils.lerp(hash(ix,iz+1),hash(ix+1,iz+1),u),v)*2-1;
}
export const lunarRocks=Array.from({length:23},()=>({x:-2.5+random()*5,z:-1.65+random()*3.3,radius:.08+random()*.095,height:.014+random()*.028,angle:random()*Math.PI,stretch:.75+random()*.5}));
const craters=[[-1.5,-2.1,.35,.065],[1,2,.3,.045],[2.6,-1.9,.4,.07],[-2.4,1.8,.32,.05],[-3.9,-3.1,1.1,.23],[3.8,3.6,1.4,.28]];
for(let i=0;i<65;i++){
 const x=-8+random()*16,z=-6+random()*12,r=.09+random()*.35;
 const onRoute=Math.abs(x)<3&&Math.abs(z)<1.7;
 craters.push([x,z,onRoute?Math.min(r,.16):r,onRoute?.006+random()*.007:.018+random()*.065]);
}
function rockRelief(rock,x,z){
 const dx=x-rock.x,dz=z-rock.z,c=Math.cos(rock.angle),s=Math.sin(rock.angle),a=dx*c+dz*s,b=(-dx*s+dz*c)/rock.stretch;
 const edge=1+.1*Math.sin(Math.atan2(b,a)*3+rock.angle)+.06*Math.cos(Math.atan2(b,a)*5);
 const q=Math.hypot(a,b)/(rock.radius*edge);
 return q<1?rock.height*(1-q*q)**1.45:0;
}
export function regolithHeight(x,z){
 let h=.02*noise(x*.65,z*.65)+.011*noise(x*2.7+18,z*2.7)+.003*noise(x*9,z*9+7);
 for(const [cx,cz,r,depth] of craters){
  const angle=Math.atan2(z-cz,x-cx),edge=1+.045*Math.sin(angle*3+cx)+.025*Math.sin(angle*7+cz);
  const q=Math.hypot(x-cx,z-cz)/(r*edge);
  if(q<1)h-=depth*(1-q*q)**2;
  if(q<1.5)h+=depth*.22*Math.exp(-(((q-1)/.13)**2));
 }return h;
}
export function lunarSurfaceHeight(x,z){
 let height=regolithHeight(x,z);
 for(const rock of lunarRocks)height+=rockRelief(rock,x,z);
 return height;
}
export function fleetPathPose(time,index){
 const slot=index*2;
 const travel=time*.045+slot*.4,phase=travel*3.4+slot*.3,vx=-.045,vz=.17*3.4*.045*Math.cos(phase);
 const yawAt=t=>{
  const lateralSpeed=.17*3.4*.045*Math.cos((t*.045+slot*.4)*3.4+slot*.3);
  return .35*Math.atan2(lateralSpeed,.045)*Math.sin(t*.12+slot*.2)**2;
 };
 return {x:2.6-((travel+5.2)%5.2),z:lanes[index]+.17*Math.sin(phase),travel,vx,vz,yaw:yawAt(time),yawRate:(yawAt(time+.005)-yawAt(time-.005))/.01};
}
export function createV3FleetMotion(robot,scene){
 const background=scene.background,fog=scene.fog;
 const environment=new THREE.Group();environment.name='Lunar regolith and rocks';scene.add(environment);
 scene.background=new THREE.Color(0x030405);scene.fog=null;
 const terrainGeo=new THREE.PlaneGeometry(20,16,400,320);terrainGeo.rotateX(-Math.PI/2);
 const p=terrainGeo.attributes.position,colors=new Float32Array(p.count*3);
 const surfaceColor=(x,z)=>{const mottling=noise(x*.9+21,z*.9)+.4*noise(x*4,z*4),shade=.27+.025*mottling;return [shade,shade*.995,shade*.98];};
 for(let i=0;i<p.count;i++){
  const x=p.getX(i),z=p.getZ(i);
  p.setY(i,regolithHeight(x,z));colors.set(surfaceColor(x,z),i*3);
 }
 terrainGeo.setAttribute('color',new THREE.BufferAttribute(colors,3));terrainGeo.computeVertexNormals();
 const textureSize=512,grainData=new Uint8Array(textureSize*textureSize*4);
 for(let i=0;i<textureSize*textureSize;i++){
  const x=i%textureSize,z=Math.floor(i/textureSize),grain=hash(x,z),dust=.84+.12*grain+.04*noise(x*.07,z*.07);
  const value=Math.round(dust*255);grainData.set([value,value,value,255],i*4);
 }
 const dustTexture=new THREE.DataTexture(grainData,textureSize,textureSize,THREE.RGBAFormat);dustTexture.wrapS=dustTexture.wrapT=THREE.RepeatWrapping;dustTexture.repeat.set(28,22);dustTexture.magFilter=THREE.LinearFilter;dustTexture.minFilter=THREE.LinearMipmapLinearFilter;dustTexture.generateMipmaps=true;dustTexture.needsUpdate=true;
 const terrain=new THREE.Mesh(terrainGeo,new THREE.MeshStandardMaterial({vertexColors:true,roughness:1,map:dustTexture,bumpMap:dustTexture,bumpScale:.0007}));terrain.receiveShadow=true;environment.add(terrain);
 for(const rock of lunarRocks){
  const geometry=new THREE.PlaneGeometry(rock.radius*3,rock.radius*3,22,22);geometry.rotateX(-Math.PI/2);
  const positions=geometry.attributes.position,uv=geometry.attributes.uv,indices=[],rockColors=new Float32Array(positions.count*3);
  for(let i=0;i<positions.count;i++){
   const x=positions.getX(i)+rock.x,z=positions.getZ(i)+rock.z;
   positions.setY(i,lunarSurfaceHeight(x,z)+.0002);
   rockColors.set(surfaceColor(x,z),i*3);
   // Continue the ground's texture coordinates across each raised patch.
   uv.setXY(i,(x+10)/20,(8-z)/16);
  }
  for(let i=0;i<geometry.index.count;i+=3){const tri=[geometry.index.getX(i),geometry.index.getX(i+1),geometry.index.getX(i+2)];if(tri.some(j=>rockRelief(rock,positions.getX(j)+rock.x,positions.getZ(j)+rock.z)>.00005))indices.push(...tri);}
  geometry.setAttribute('color',new THREE.BufferAttribute(rockColors,3));geometry.setIndex(indices);geometry.computeVertexNormals();const mesh=new THREE.Mesh(geometry,terrain.material);mesh.position.set(rock.x,0,rock.z);mesh.receiveShadow=true;environment.add(mesh);
 }
 // Irregular angular ejecta and boulders, outside the rovers' contact corridor.
 const boulderMaterial=new THREE.MeshStandardMaterial({color:0x646361,roughness:1,flatShading:true,map:dustTexture});
 for(let i=0;i<95;i++){
  const x=-8+random()*16,z=-6+random()*12;if(Math.abs(x)<3.15&&Math.abs(z)<1.85)continue;
  const radius=.025+random()**2*.23,geometry=new THREE.IcosahedronGeometry(radius,1),vertices=geometry.attributes.position;
  for(let j=0;j<vertices.count;j++){const vx=vertices.getX(j),vy=vertices.getY(j),vz=vertices.getZ(j),roughness=.85+.25*noise(vx*40+3,vz*40+vy*29);vertices.setXYZ(j,vx*roughness,vy*roughness,vz*roughness);}
  geometry.computeVertexNormals();const mesh=new THREE.Mesh(geometry,boulderMaterial);mesh.scale.set(.7+random()*.65,.45+random()*.65,.7+random()*.6);mesh.rotation.y=random()*Math.PI;mesh.position.set(x,regolithHeight(x,z)+radius*.25,z);mesh.castShadow=true;mesh.receiveShadow=true;environment.add(mesh);
 }
 const replicas=[robot,...Array.from({length:lanes.length-1},()=>robot.clone())];
 replicas.slice(1).forEach(r=>scene.add(r));
 const controllers=replicas.map(r=>createV3Suspension(r,lunarSurfaceHeight));
 // Clone geometry and material references are shared. Render identical parts together,
 // while the original full-resolution meshes remain in the URDF for kinematics.
 const batches=new Map(),sourceMeshes=[];
 for(const replica of replicas)replica.traverse(mesh=>{
  if(!mesh.isMesh)return;
  const key=`${mesh.geometry.id}:${mesh.material.id}`;
  if(!batches.has(key))batches.set(key,{geometry:mesh.geometry,material:mesh.material,sources:[]});
  batches.get(key).sources.push(mesh);sourceMeshes.push(mesh);mesh.visible=false;
 });
 for(const batch of batches.values()){
  batch.mesh=new THREE.InstancedMesh(batch.geometry,batch.material,batch.sources.length);
  // Keep fine CAD faces free of shadow-map streaks; the fleet uses contact
  // shadows beneath each rover, while the lunar terrain retains its shadows.
  batch.mesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);batch.mesh.frustumCulled=false;batch.mesh.receiveShadow=false;scene.add(batch.mesh);
 }
 const shadowSize=64,shadowData=new Uint8Array(shadowSize*shadowSize*4);
 for(let y=0;y<shadowSize;y++)for(let x=0;x<shadowSize;x++){
  const q=Math.hypot((x+.5)/shadowSize*2-1,(y+.5)/shadowSize*2-1),alpha=Math.max(0,1-q*q)**2*.27;
  shadowData.set([0,0,0,Math.round(alpha*255)],(y*shadowSize+x)*4);
 }
 const shadowTexture=new THREE.DataTexture(shadowData,shadowSize,shadowSize,THREE.RGBAFormat);shadowTexture.magFilter=THREE.LinearFilter;shadowTexture.needsUpdate=true;
 const shadowMaterial=new THREE.MeshBasicMaterial({map:shadowTexture,transparent:true,depthWrite:false,polygonOffset:true,polygonOffsetFactor:-1});
 const contactShadows=replicas.map(()=>{
  const geometry=new THREE.PlaneGeometry(.54,.42,6,4);geometry.rotateX(-Math.PI/2);
  const mesh=new THREE.Mesh(geometry,shadowMaterial);environment.add(mesh);return mesh;
 });
 const sun=new THREE.DirectionalLight(0xffffff,2);sun.position.set(-3,6,2);sun.castShadow=true;sun.shadow.mapSize.set(1024,1024);sun.shadow.camera.left=-4;sun.shadow.camera.right=4;sun.shadow.camera.top=3;sun.shadow.camera.bottom=-3;sun.shadow.camera.near=.1;sun.shadow.camera.far=15;sun.shadow.bias=-.0003;scene.add(sun,sun.target);
 sun.shadow.autoUpdate=false;sun.shadow.needsUpdate=true;
 let elapsed=0;
 function pose(delta=0){
  controllers.forEach((c,i)=>{
   const p=fleetPathPose(elapsed,i);c.pose(p.x,p.z,p.yaw,-p.travel,{vx:p.vx,vz:p.vz,yawRate:p.yawRate,delta});
   const shadow=contactShadows[i],vertices=shadow.geometry.attributes.position;
   shadow.position.set(p.x,0,p.z);shadow.rotation.y=p.yaw;
   for(let j=0;j<vertices.count;j++){
    const x=vertices.getX(j),z=vertices.getZ(j),wx=p.x+x*Math.cos(p.yaw)+z*Math.sin(p.yaw),wz=p.z-x*Math.sin(p.yaw)+z*Math.cos(p.yaw);
    vertices.setY(j,lunarSurfaceHeight(wx,wz)+.001);
   }vertices.needsUpdate=true;
  });
  for(const batch of batches.values()){
   batch.sources.forEach((mesh,index)=>batch.mesh.setMatrixAt(index,mesh.matrixWorld));batch.mesh.instanceMatrix.needsUpdate=true;
  }
 }
 pose();
 return {
  cameraTarget:new THREE.Vector3(0,.25,0),viewScale:6.5,
  description:'Three V3 rovers steer and crab across rough lunar terrain. Each wheel follows the local direction and speed of travel while the suspension maintains both differential linkage closures.',
  get phase(){return 'Three V3 rovers traversing lunar terrain';},
  update(time,delta){elapsed+=delta;pose(delta);},
  reset(){elapsed=0;controllers.forEach(c=>c.resetDrive());pose();},
  diagnostics(){return controllers.map(c=>c.diagnostics());},
  dispose(){replicas.slice(1).forEach(r=>scene.remove(r));for(const batch of batches.values()){scene.remove(batch.mesh);batch.mesh.dispose();}sourceMeshes.forEach(m=>{m.visible=true;});scene.remove(environment,sun,sun.target);environment.traverse(c=>{c.geometry?.dispose();c.material?.dispose();});dustTexture.dispose();shadowTexture.dispose();scene.background=background;scene.fog=fog;}
 };
}
