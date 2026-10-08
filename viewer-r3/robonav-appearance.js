import * as THREE from 'three';
import { robonavCADMaterials } from './robonav-cad-materials.js?v=20261008-animation2';
export function robonavMaterialForMesh(url,fallback){
  const name=decodeURIComponent(url.split(/[?#]/)[0].split('/').pop()).replace(/\.stl$/i,'');
  const source=robonavCADMaterials[name];
  if(!source)return fallback;
  const [r,g,b,a=1]=source.baseColorFactor;
  return new THREE.MeshStandardMaterial({name:`RoboNav CAD — ${name}`,color:new THREE.Color().setRGB(r,g,b,THREE.LinearSRGBColorSpace),metalness:source.metallicFactor??0,roughness:source.roughnessFactor??1,opacity:a,transparent:a<1});
}
