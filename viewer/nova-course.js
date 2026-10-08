// A repeatable obstacle course: stop before steering sideways, clear the block,
// then return to the original lane. Distances are metres and times are seconds.
export const courseLength=3.6;
export const rocks=[{f:.36,s:.13,r:.14,h:.035},{f:.65,s:-.13,r:.15,h:.045},{f:.84,s:.13,r:.12,h:.03}];
export const block={f:1.65,s:0,length:.44,width:.4,height:.38};
export function rockSurface(rock,df,ds){
 const angle=Math.atan2(ds,df);
 const edge=1+.12*Math.sin(angle*3+rock.f*9)+.07*Math.cos(angle*5);
 const q=(df*df+(ds*1.12)**2)/(rock.r*edge)**2;
 if(q>=1)return 0;
 return rock.h*(1-q)**1.5*(.9+.1*Math.sin(df*42+ds*27+rock.f));
}
// Shallow regolith relief and crater bowls beside the driving corridor.
export function lunarHeight(f,s){
 let h=.0005*Math.sin(f*47+s*29)*Math.sin(f*31-s*53);
 for(const [cf,cs,r,depth] of [[.4,-.58,.24,.05],[1.55,-.65,.33,.065],[2.6,1.13,.24,.04],[3.15,-.5,.17,.03]]){
  const df=f-cf-Math.round((f-cf)/courseLength)*courseLength;
  const q=Math.hypot(df,s-cs)/r;
  if(q<1.3)h+=q<1?-depth*(1-q*q)**2:0;
  if(q<1.3)h+=depth*.23*Math.exp(-(((q-1)/.13)**2));
 }
 return h;
}
const stages=[
  [14,1.05,0,0,'Crossing rocks'],
  [1.5,1.05,0,Math.PI/2,'Stopping and steering sideways'],
  [7,1.05,.68,Math.PI/2,'Crab sideways around the block'],
  [1.5,1.05,.68,0,'Aligning wheels forward'],
  [13,2.25,.68,0,'Passing the block'],
  [1.5,2.25,.68,-Math.PI/2,'Steering back toward the route'],
  [7,2.25,0,-Math.PI/2,'Returning to the original lane'],
  [1.5,2.25,0,0,'Aligning wheels forward'],
  [16,courseLength,0,0,'Continuing forward']
];
export const courseDuration=stages.reduce((sum,s)=>sum+s[0],0);
export function coursePose(time){
 const cycle=Math.floor(time/courseDuration);let t=time-cycle*courseDuration,previous=[0,0,0];
 for(const [duration,f,s,angle,label] of stages){
  if(t<=duration){const u=Math.max(0,t/duration),ease=u*u*(3-2*u),rate=6*u*(1-u)/duration;
   return {distance:cycle*courseLength+previous[0]+(f-previous[0])*ease,lateral:previous[1]+(s-previous[1])*ease,angle:previous[2]+(angle-previous[2])*ease,speed:Math.hypot(f-previous[0],s-previous[1])*rate,label};
  }t-=duration;previous=[f,s,angle];
 }
}
export function rockHeight(f,s){
 let height=0;
 for(const rock of rocks){const df=f-rock.f-Math.round((f-rock.f)/courseLength)*courseLength;height=Math.max(height,rockSurface(rock,df,s-rock.s));}
 return height;
}
