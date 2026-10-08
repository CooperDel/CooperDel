// Share a bounded request queue across every model viewer on the page.
export function createMeshQueue(limit=8){
  let active=0;const waiting=[];
  function pump(){while(active<limit&&waiting.length){const {task,resolve,reject}=waiting.shift();active++;Promise.resolve().then(task).then(resolve,reject).finally(()=>{active--;pump();});}}
  return task=>new Promise((resolve,reject)=>{waiting.push({task,resolve,reject});pump();});
}
export const queueMesh=createMeshQueue();
