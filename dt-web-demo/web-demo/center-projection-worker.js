/* One requested center projection per worker; no shared regional state. */
importScripts('center-model.js','center-projection.js');
self.onmessage=event=>{const {id,input,policy}=event.data;try{self.postMessage({id,result:CenterProjection.compare(input,policy)});}catch(error){self.postMessage({id,error:error.message});}};
