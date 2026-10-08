import * as THREE from 'three';

// Rest-pose constraints preserve the small CAD-export offsets instead of snapping parts.
export function createUUVManipulator(robot) {
  const config=robot.userData.parallelManipulator;
  if(!config)throw new Error('UUV parallel-manipulator closure metadata is missing.');
  const carrier=robot.joints[config.carrier];
  const joints=Object.values(robot.joints).filter(j=>j.jointType==='continuous'&&!j.name.includes('thruster'));
  if(!carrier || carrier.jointType!=='floating' || joints.length!==21)throw new Error('Unexpected UUV parallel joint structure.');
  const active=[carrier,...joints], index=new Map(active.map((j,i)=>[j,i===0?0:6+i-1]));
  const original=active.map(j=>({values:j.jointValue.slice(),position:j.position.clone(),quaternion:j.quaternion.clone()}));
  const state=new Array(27).fill(0);
  robot.updateMatrixWorld(true);
  const paths=new Map();
  function path(name){
    if(paths.has(name))return paths.get(name);
    let object=robot.links[name];if(!object)throw new Error(`Missing UUV link ${name}`);
    const chain=[];while(object && object!==robot){chain.unshift(object);object=object.parent;}
    const segments=[];let fixed=new THREE.Matrix4();
    for(const node of chain){
      node.updateMatrix();
      if(index.has(node)){
        if(node===carrier){segments.push({fixed:fixed.clone()});segments.push({joint:node,origin:node.matrix.clone(),offset:0});}
        else segments.push({joint:node,origin:fixed.clone().multiply(node.matrix),offset:index.get(node)});
        fixed.identity();
      }else fixed.multiply(node.matrix);
    }
    segments.push({fixed:fixed.clone()});paths.set(name,segments);return segments;
  }
  const rotation=new THREE.Quaternion(), transform=new THREE.Matrix4(), xyz=new THREE.Vector3(), unit=new THREE.Vector3(1,1,1), euler=new THREE.Euler(0,0,0,'XYZ');
  function matrix(name,q){
    const result=new THREE.Matrix4();
    for(const part of path(name)){
      if(part.fixed){result.multiply(part.fixed);continue;}
      if(part.joint===carrier){
        xyz.set(q[0],q[1],q[2]);rotation.setFromEuler(euler.set(q[3],q[4],q[5],'XYZ'));
        result.multiply(transform.compose(xyz,rotation,unit)).multiply(part.origin);
      }else{
        rotation.setFromAxisAngle(part.joint.axis,q[part.offset]);
        result.multiply(part.origin).multiply(transform.makeRotationFromQuaternion(rotation));
      }
    }
    return result;
  }
  const anchors=config.anchors.map(name=>({name,rest:matrix(name,state)}));
  const closures=config.closures.map(([a,b])=>({a,b,rest:matrix(a,state).invert().multiply(matrix(b,state))}));
  const home=matrix(config.endEffector,state), homePosition=new THREE.Vector3().setFromMatrixPosition(home);
  let desired=home.clone(), lastError=0, lastTargetError=0;
  function difference(actual,expected,out){
    const d=expected.clone().invert().multiply(actual),p=new THREE.Vector3().setFromMatrixPosition(d),q=new THREE.Quaternion().setFromRotationMatrix(d);
    const sign=q.w<0?-1:1;
    out.push(p.x*10,p.y*10,p.z*10,2*q.x*sign,2*q.y*sign,2*q.z*sign);
  }
  function residual(q){
    const out=[];
    for(const a of anchors)difference(matrix(a.name,q),a.rest,out);
    for(const c of closures)difference(matrix(c.a,q).invert().multiply(matrix(c.b,q)),c.rest,out);
    difference(matrix(config.endEffector,q),desired,out);
    return out;
  }
  function linearSolve(A,b){
    const n=b.length;
    for(let i=0;i<n;i++){
      let pivot=i;for(let k=i+1;k<n;k++)if(Math.abs(A[k][i])>Math.abs(A[pivot][i]))pivot=k;
      [A[i],A[pivot]]=[A[pivot],A[i]];[b[i],b[pivot]]=[b[pivot],b[i]];
      if(Math.abs(A[i][i])<1e-14)return null;
      for(let k=i+1;k<n;k++){const f=A[k][i]/A[i][i];for(let j=i+1;j<n;j++)A[k][j]-=f*A[i][j];b[k]-=f*b[i];}
    }
    const x=new Array(n).fill(0);
    for(let i=n-1;i>=0;i--){let sum=b[i];for(let j=i+1;j<n;j++)sum-=A[i][j]*x[j];x[i]=sum/A[i][i];}return x;
  }
  function apply(){
    carrier.setJointValue(...state.slice(0,6));
    // Keep the floating transform in quaternion form: this export sits near an
    // Euler singularity and the loader's matrix-to-Euler round trip loses precision.
    rotation.setFromEuler(euler.set(state[3],state[4],state[5],'XYZ'));
    carrier.position.copy(carrier.origPosition).applyQuaternion(rotation).add(xyz.set(state[0],state[1],state[2]));
    carrier.quaternion.copy(rotation).multiply(carrier.origQuaternion);
    carrier.matrixWorldNeedsUpdate=true;
    joints.forEach((j,i)=>j.setJointValue(state[i+6]));
  }
  function solve(target){
    desired=target;const previous=state.slice();
    for(let iteration=0;iteration<10;iteration++){
      const r=residual(state),cost=r.reduce((s,v)=>s+v*v,0);
      if(Math.max(...r.map(Math.abs))<2e-6)break;
      const J=state.map((v,i)=>{state[i]=v+1e-5;const next=residual(state);state[i]=v;return next.map((n,k)=>(n-r[k])/1e-5);});
      const A=J.map((column,i)=>J.map((other,j)=>column.reduce((s,v,k)=>s+v*other[k],0)+(i===j?1e-7:0)));
      const b=J.map(column=>-column.reduce((s,v,k)=>s+v*r[k],0));
      const step=linearSolve(A,b);if(!step)break;
      let improved=false;
      for(const scale of [1,.5,.25,.125]){
        const candidate=state.map((v,i)=>v+Math.max(-.15,Math.min(.15,step[i]))*scale);
        const next=residual(candidate);
        if(next.reduce((s,v)=>s+v*v,0)<cost){state.splice(0,state.length,...candidate);improved=true;break;}
      }
      if(!improved)break;
    }
    const r=residual(state);lastError=Math.max(...r.slice(0,-6).map(Math.abs));lastTargetError=Math.max(...r.slice(-6).map(Math.abs));
    // Never display a broken linkage if a target lies outside the reachable branch.
    if(lastError>2e-4 || lastTargetError>2e-4 || !state.every(Number.isFinite)){state.splice(0,state.length,...previous);return false;}
    apply();return true;
  }
  return {
    get homePosition(){return homePosition.clone();},
    moveTo(position){return solve(home.clone().setPosition(position));},
    update(time){
      const t=time*.65, offset=new THREE.Vector3(.018*Math.sin(t),-.012*(1-Math.cos(t)),.012*Math.sin(2*t));
      const target=home.clone().setPosition(homePosition.clone().add(offset));return solve(target);
    },
    reset(){state.fill(0);desired=home.clone();lastError=0;lastTargetError=0;apply();},
    get error(){return lastError;},get targetError(){return lastTargetError;},
    get jointValues(){return state.slice();},
    get position(){return new THREE.Vector3().setFromMatrixPosition(matrix(config.endEffector,state));},
    dispose(){active.forEach((j,i)=>{j.setJointValue(...original[i].values);j.position.copy(original[i].position);j.quaternion.copy(original[i].quaternion);j.matrixWorldNeedsUpdate=true;});}
  };
}
