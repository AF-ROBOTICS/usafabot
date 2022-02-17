(function(){/*

 Copyright The Closure Library Authors.
 SPDX-License-Identifier: Apache-2.0
*/
'use strict';var m;function ba(a){var b=0;return function(){return b<a.length?{done:!1,value:a[b++]}:{done:!0}}}
var ca="function"==typeof Object.defineProperties?Object.defineProperty:function(a,b,c){if(a==Array.prototype||a==Object.prototype)return a;a[b]=c.value;return a};
function da(a){a=["object"==typeof globalThis&&globalThis,a,"object"==typeof window&&window,"object"==typeof self&&self,"object"==typeof global&&global];for(var b=0;b<a.length;++b){var c=a[b];if(c&&c.Math==Math)return c}throw Error("Cannot find global object");}
var fa=da(this);function p(a,b){if(b)a:{var c=fa;a=a.split(".");for(var d=0;d<a.length-1;d++){var e=a[d];if(!(e in c))break a;c=c[e]}a=a[a.length-1];d=c[a];b=b(d);b!=d&&null!=b&&ca(c,a,{configurable:!0,writable:!0,value:b})}}
p("Symbol",function(a){function b(f){if(this instanceof b)throw new TypeError("Symbol is not a constructor");return new c(d+(f||"")+"_"+e++,f)}
function c(f,g){this.h=f;ca(this,"description",{configurable:!0,writable:!0,value:g})}
if(a)return a;c.prototype.toString=function(){return this.h};
var d="jscomp_symbol_"+(1E9*Math.random()>>>0)+"_",e=0;return b});
p("Symbol.iterator",function(a){if(a)return a;a=Symbol("Symbol.iterator");for(var b="Array Int8Array Uint8Array Uint8ClampedArray Int16Array Uint16Array Int32Array Uint32Array Float32Array Float64Array".split(" "),c=0;c<b.length;c++){var d=fa[b[c]];"function"===typeof d&&"function"!=typeof d.prototype[a]&&ca(d.prototype,a,{configurable:!0,writable:!0,value:function(){return ha(ba(this))}})}return a});
function ha(a){a={next:a};a[Symbol.iterator]=function(){return this};
return a}
function q(a){var b="undefined"!=typeof Symbol&&Symbol.iterator&&a[Symbol.iterator];return b?b.call(a):{next:ba(a)}}
function ia(a){if(!(a instanceof Array)){a=q(a);for(var b,c=[];!(b=a.next()).done;)c.push(b.value);a=c}return a}
var ka="function"==typeof Object.create?Object.create:function(a){function b(){}
b.prototype=a;return new b},la=function(){function a(){function c(){}
new c;Reflect.construct(c,[],function(){});
return new c instanceof c}
if("undefined"!=typeof Reflect&&Reflect.construct){if(a())return Reflect.construct;var b=Reflect.construct;return function(c,d,e){c=b(c,d);e&&Reflect.setPrototypeOf(c,e.prototype);return c}}return function(c,d,e){void 0===e&&(e=c);
e=ka(e.prototype||Object.prototype);return Function.prototype.apply.call(c,e,d)||e}}(),ma;
if("function"==typeof Object.setPrototypeOf)ma=Object.setPrototypeOf;else{var na;a:{var oa={a:!0},pa={};try{pa.__proto__=oa;na=pa.a;break a}catch(a){}na=!1}ma=na?function(a,b){a.__proto__=b;if(a.__proto__!==b)throw new TypeError(a+" is not extensible");return a}:null}var qa=ma;
function r(a,b){a.prototype=ka(b.prototype);a.prototype.constructor=a;if(qa)qa(a,b);else for(var c in b)if("prototype"!=c)if(Object.defineProperties){var d=Object.getOwnPropertyDescriptor(b,c);d&&Object.defineProperty(a,c,d)}else a[c]=b[c];a.aa=b.prototype}
function ra(){this.D=!1;this.l=null;this.i=void 0;this.h=1;this.o=this.m=0;this.A=this.j=null}
function sa(a){if(a.D)throw new TypeError("Generator is already running");a.D=!0}
ra.prototype.u=function(a){this.i=a};
function ta(a,b){a.j={vb:b,zb:!0};a.h=a.m||a.o}
ra.prototype.return=function(a){this.j={return:a};this.h=this.o};
function v(a,b,c){a.h=c;return{value:b}}
ra.prototype.s=function(a){this.h=a};
function ua(a,b,c){a.m=b;void 0!=c&&(a.o=c)}
function va(a,b){a.h=b;a.m=0}
function wa(a){a.m=0;var b=a.j.vb;a.j=null;return b}
function xa(a){a.A=[a.j];a.m=0;a.o=0}
function ya(a){var b=a.A.splice(0)[0];(b=a.j=a.j||b)?b.zb?a.h=a.m||a.o:void 0!=b.s&&a.o<b.s?(a.h=b.s,a.j=null):a.h=a.o:a.h=0}
function za(a){this.h=new ra;this.i=a}
function Aa(a,b){sa(a.h);var c=a.h.l;if(c)return Ba(a,"return"in c?c["return"]:function(d){return{value:d,done:!0}},b,a.h.return);
a.h.return(b);return Ca(a)}
function Ba(a,b,c,d){try{var e=b.call(a.h.l,c);if(!(e instanceof Object))throw new TypeError("Iterator result "+e+" is not an object");if(!e.done)return a.h.D=!1,e;var f=e.value}catch(g){return a.h.l=null,ta(a.h,g),Ca(a)}a.h.l=null;d.call(a.h,f);return Ca(a)}
function Ca(a){for(;a.h.h;)try{var b=a.i(a.h);if(b)return a.h.D=!1,{value:b.value,done:!1}}catch(c){a.h.i=void 0,ta(a.h,c)}a.h.D=!1;if(a.h.j){b=a.h.j;a.h.j=null;if(b.zb)throw b.vb;return{value:b.return,done:!0}}return{value:void 0,done:!0}}
function Da(a){this.next=function(b){sa(a.h);a.h.l?b=Ba(a,a.h.l.next,b,a.h.u):(a.h.u(b),b=Ca(a));return b};
this.throw=function(b){sa(a.h);a.h.l?b=Ba(a,a.h.l["throw"],b,a.h.u):(ta(a.h,b),b=Ca(a));return b};
this.return=function(b){return Aa(a,b)};
this[Symbol.iterator]=function(){return this}}
function Ea(a){function b(d){return a.next(d)}
function c(d){return a.throw(d)}
return new Promise(function(d,e){function f(g){g.done?d(g.value):Promise.resolve(g.value).then(b,c).then(f,e)}
f(a.next())})}
function w(a){return Ea(new Da(new za(a)))}
function Fa(){for(var a=Number(this),b=[],c=a;c<arguments.length;c++)b[c-a]=arguments[c];return b}
p("Reflect",function(a){return a?a:{}});
p("Reflect.construct",function(){return la});
p("Reflect.setPrototypeOf",function(a){return a?a:qa?function(b,c){try{return qa(b,c),!0}catch(d){return!1}}:null});
p("Promise",function(a){function b(g){this.h=0;this.j=void 0;this.i=[];this.D=!1;var h=this.l();try{g(h.resolve,h.reject)}catch(k){h.reject(k)}}
function c(){this.h=null}
function d(g){return g instanceof b?g:new b(function(h){h(g)})}
if(a)return a;c.prototype.i=function(g){if(null==this.h){this.h=[];var h=this;this.j(function(){h.o()})}this.h.push(g)};
var e=fa.setTimeout;c.prototype.j=function(g){e(g,0)};
c.prototype.o=function(){for(;this.h&&this.h.length;){var g=this.h;this.h=[];for(var h=0;h<g.length;++h){var k=g[h];g[h]=null;try{k()}catch(l){this.l(l)}}}this.h=null};
c.prototype.l=function(g){this.j(function(){throw g;})};
b.prototype.l=function(){function g(l){return function(n){k||(k=!0,l.call(h,n))}}
var h=this,k=!1;return{resolve:g(this.S),reject:g(this.o)}};
b.prototype.S=function(g){if(g===this)this.o(new TypeError("A Promise cannot resolve to itself"));else if(g instanceof b)this.ga(g);else{a:switch(typeof g){case "object":var h=null!=g;break a;case "function":h=!0;break a;default:h=!1}h?this.M(g):this.m(g)}};
b.prototype.M=function(g){var h=void 0;try{h=g.then}catch(k){this.o(k);return}"function"==typeof h?this.oa(h,g):this.m(g)};
b.prototype.o=function(g){this.u(2,g)};
b.prototype.m=function(g){this.u(1,g)};
b.prototype.u=function(g,h){if(0!=this.h)throw Error("Cannot settle("+g+", "+h+"): Promise already settled in state"+this.h);this.h=g;this.j=h;2===this.h&&this.Z();this.A()};
b.prototype.Z=function(){var g=this;e(function(){if(g.L()){var h=fa.console;"undefined"!==typeof h&&h.error(g.j)}},1)};
b.prototype.L=function(){if(this.D)return!1;var g=fa.CustomEvent,h=fa.Event,k=fa.dispatchEvent;if("undefined"===typeof k)return!0;"function"===typeof g?g=new g("unhandledrejection",{cancelable:!0}):"function"===typeof h?g=new h("unhandledrejection",{cancelable:!0}):(g=fa.document.createEvent("CustomEvent"),g.initCustomEvent("unhandledrejection",!1,!0,g));g.promise=this;g.reason=this.j;return k(g)};
b.prototype.A=function(){if(null!=this.i){for(var g=0;g<this.i.length;++g)f.i(this.i[g]);this.i=null}};
var f=new c;b.prototype.ga=function(g){var h=this.l();g.Ja(h.resolve,h.reject)};
b.prototype.oa=function(g,h){var k=this.l();try{g.call(h,k.resolve,k.reject)}catch(l){k.reject(l)}};
b.prototype.then=function(g,h){function k(y,u){return"function"==typeof y?function(D){try{l(y(D))}catch(E){n(E)}}:u}
var l,n,t=new b(function(y,u){l=y;n=u});
this.Ja(k(g,l),k(h,n));return t};
b.prototype.catch=function(g){return this.then(void 0,g)};
b.prototype.Ja=function(g,h){function k(){switch(l.h){case 1:g(l.j);break;case 2:h(l.j);break;default:throw Error("Unexpected state: "+l.h);}}
var l=this;null==this.i?f.i(k):this.i.push(k);this.D=!0};
b.resolve=d;b.reject=function(g){return new b(function(h,k){k(g)})};
b.race=function(g){return new b(function(h,k){for(var l=q(g),n=l.next();!n.done;n=l.next())d(n.value).Ja(h,k)})};
b.all=function(g){var h=q(g),k=h.next();return k.done?d([]):new b(function(l,n){function t(D){return function(E){y[D]=E;u--;0==u&&l(y)}}
var y=[],u=0;do y.push(void 0),u++,d(k.value).Ja(t(y.length-1),n),k=h.next();while(!k.done)})};
return b});
function Ha(a,b){return Object.prototype.hasOwnProperty.call(a,b)}
p("WeakMap",function(a){function b(k){this.h=(h+=Math.random()+1).toString();if(k){k=q(k);for(var l;!(l=k.next()).done;)l=l.value,this.set(l[0],l[1])}}
function c(){}
function d(k){var l=typeof k;return"object"===l&&null!==k||"function"===l}
function e(k){if(!Ha(k,g)){var l=new c;ca(k,g,{value:l})}}
function f(k){var l=Object[k];l&&(Object[k]=function(n){if(n instanceof c)return n;Object.isExtensible(n)&&e(n);return l(n)})}
if(function(){if(!a||!Object.seal)return!1;try{var k=Object.seal({}),l=Object.seal({}),n=new a([[k,2],[l,3]]);if(2!=n.get(k)||3!=n.get(l))return!1;n.delete(k);n.set(l,4);return!n.has(k)&&4==n.get(l)}catch(t){return!1}}())return a;
var g="$jscomp_hidden_"+Math.random();f("freeze");f("preventExtensions");f("seal");var h=0;b.prototype.set=function(k,l){if(!d(k))throw Error("Invalid WeakMap key");e(k);if(!Ha(k,g))throw Error("WeakMap key fail: "+k);k[g][this.h]=l;return this};
b.prototype.get=function(k){return d(k)&&Ha(k,g)?k[g][this.h]:void 0};
b.prototype.has=function(k){return d(k)&&Ha(k,g)&&Ha(k[g],this.h)};
b.prototype.delete=function(k){return d(k)&&Ha(k,g)&&Ha(k[g],this.h)?delete k[g][this.h]:!1};
return b});
p("Map",function(a){function b(){var h={};return h.previous=h.next=h.head=h}
function c(h,k){var l=h.h;return ha(function(){if(l){for(;l.head!=h.h;)l=l.previous;for(;l.next!=l.head;)return l=l.next,{done:!1,value:k(l)};l=null}return{done:!0,value:void 0}})}
function d(h,k){var l=k&&typeof k;"object"==l||"function"==l?f.has(k)?l=f.get(k):(l=""+ ++g,f.set(k,l)):l="p_"+k;var n=h.data_[l];if(n&&Ha(h.data_,l))for(h=0;h<n.length;h++){var t=n[h];if(k!==k&&t.key!==t.key||k===t.key)return{id:l,list:n,index:h,entry:t}}return{id:l,list:n,index:-1,entry:void 0}}
function e(h){this.data_={};this.h=b();this.size=0;if(h){h=q(h);for(var k;!(k=h.next()).done;)k=k.value,this.set(k[0],k[1])}}
if(function(){if(!a||"function"!=typeof a||!a.prototype.entries||"function"!=typeof Object.seal)return!1;try{var h=Object.seal({x:4}),k=new a(q([[h,"s"]]));if("s"!=k.get(h)||1!=k.size||k.get({x:4})||k.set({x:4},"t")!=k||2!=k.size)return!1;var l=k.entries(),n=l.next();if(n.done||n.value[0]!=h||"s"!=n.value[1])return!1;n=l.next();return n.done||4!=n.value[0].x||"t"!=n.value[1]||!l.next().done?!1:!0}catch(t){return!1}}())return a;
var f=new WeakMap;e.prototype.set=function(h,k){h=0===h?0:h;var l=d(this,h);l.list||(l.list=this.data_[l.id]=[]);l.entry?l.entry.value=k:(l.entry={next:this.h,previous:this.h.previous,head:this.h,key:h,value:k},l.list.push(l.entry),this.h.previous.next=l.entry,this.h.previous=l.entry,this.size++);return this};
e.prototype.delete=function(h){h=d(this,h);return h.entry&&h.list?(h.list.splice(h.index,1),h.list.length||delete this.data_[h.id],h.entry.previous.next=h.entry.next,h.entry.next.previous=h.entry.previous,h.entry.head=null,this.size--,!0):!1};
e.prototype.clear=function(){this.data_={};this.h=this.h.previous=b();this.size=0};
e.prototype.has=function(h){return!!d(this,h).entry};
e.prototype.get=function(h){return(h=d(this,h).entry)&&h.value};
e.prototype.entries=function(){return c(this,function(h){return[h.key,h.value]})};
e.prototype.keys=function(){return c(this,function(h){return h.key})};
e.prototype.values=function(){return c(this,function(h){return h.value})};
e.prototype.forEach=function(h,k){for(var l=this.entries(),n;!(n=l.next()).done;)n=n.value,h.call(k,n[1],n[0],this)};
e.prototype[Symbol.iterator]=e.prototype.entries;var g=0;return e});
function Ia(a,b,c){if(null==a)throw new TypeError("The 'this' value for String.prototype."+c+" must not be null or undefined");if(b instanceof RegExp)throw new TypeError("First argument to String.prototype."+c+" must not be a regular expression");return a+""}
p("String.prototype.endsWith",function(a){return a?a:function(b,c){var d=Ia(this,b,"endsWith");b+="";void 0===c&&(c=d.length);c=Math.max(0,Math.min(c|0,d.length));for(var e=b.length;0<e&&0<c;)if(d[--c]!=b[--e])return!1;return 0>=e}});
p("Array.prototype.find",function(a){return a?a:function(b,c){a:{var d=this;d instanceof String&&(d=String(d));for(var e=d.length,f=0;f<e;f++){var g=d[f];if(b.call(c,g,f,d)){b=g;break a}}b=void 0}return b}});
p("String.prototype.startsWith",function(a){return a?a:function(b,c){var d=Ia(this,b,"startsWith");b+="";var e=d.length,f=b.length;c=Math.max(0,Math.min(c|0,d.length));for(var g=0;g<f&&c<e;)if(d[c++]!=b[g++])return!1;return g>=f}});
function Ja(a,b){a instanceof String&&(a+="");var c=0,d=!1,e={next:function(){if(!d&&c<a.length){var f=c++;return{value:b(f,a[f]),done:!1}}d=!0;return{done:!0,value:void 0}}};
e[Symbol.iterator]=function(){return e};
return e}
p("Array.prototype.entries",function(a){return a?a:function(){return Ja(this,function(b,c){return[b,c]})}});
p("Object.setPrototypeOf",function(a){return a||qa});
var Ka="function"==typeof Object.assign?Object.assign:function(a,b){for(var c=1;c<arguments.length;c++){var d=arguments[c];if(d)for(var e in d)Ha(d,e)&&(a[e]=d[e])}return a};
p("Object.assign",function(a){return a||Ka});
p("Set",function(a){function b(c){this.h=new Map;if(c){c=q(c);for(var d;!(d=c.next()).done;)this.add(d.value)}this.size=this.h.size}
if(function(){if(!a||"function"!=typeof a||!a.prototype.entries||"function"!=typeof Object.seal)return!1;try{var c=Object.seal({x:4}),d=new a(q([c]));if(!d.has(c)||1!=d.size||d.add(c)!=d||1!=d.size||d.add({x:4})!=d||2!=d.size)return!1;var e=d.entries(),f=e.next();if(f.done||f.value[0]!=c||f.value[1]!=c)return!1;f=e.next();return f.done||f.value[0]==c||4!=f.value[0].x||f.value[1]!=f.value[0]?!1:e.next().done}catch(g){return!1}}())return a;
b.prototype.add=function(c){c=0===c?0:c;this.h.set(c,c);this.size=this.h.size;return this};
b.prototype.delete=function(c){c=this.h.delete(c);this.size=this.h.size;return c};
b.prototype.clear=function(){this.h.clear();this.size=0};
b.prototype.has=function(c){return this.h.has(c)};
b.prototype.entries=function(){return this.h.entries()};
b.prototype.values=function(){return this.h.values()};
b.prototype.keys=b.prototype.values;b.prototype[Symbol.iterator]=b.prototype.values;b.prototype.forEach=function(c,d){var e=this;this.h.forEach(function(f){return c.call(d,f,f,e)})};
return b});
p("Object.entries",function(a){return a?a:function(b){var c=[],d;for(d in b)Ha(b,d)&&c.push([d,b[d]]);return c}});
p("Array.prototype.keys",function(a){return a?a:function(){return Ja(this,function(b){return b})}});
p("Array.prototype.values",function(a){return a?a:function(){return Ja(this,function(b,c){return c})}});
p("Object.is",function(a){return a?a:function(b,c){return b===c?0!==b||1/b===1/c:b!==b&&c!==c}});
p("Array.prototype.includes",function(a){return a?a:function(b,c){var d=this;d instanceof String&&(d=String(d));var e=d.length;c=c||0;for(0>c&&(c=Math.max(c+e,0));c<e;c++){var f=d[c];if(f===b||Object.is(f,b))return!0}return!1}});
p("String.prototype.includes",function(a){return a?a:function(b,c){return-1!==Ia(this,b,"includes").indexOf(b,c||0)}});
p("Array.from",function(a){return a?a:function(b,c,d){c=null!=c?c:function(h){return h};
var e=[],f="undefined"!=typeof Symbol&&Symbol.iterator&&b[Symbol.iterator];if("function"==typeof f){b=f.call(b);for(var g=0;!(f=b.next()).done;)e.push(c.call(d,f.value,g++))}else for(f=b.length,g=0;g<f;g++)e.push(c.call(d,b[g],g));return e}});
p("Number.isNaN",function(a){return a?a:function(b){return"number"===typeof b&&isNaN(b)}});
p("Number.MAX_SAFE_INTEGER",function(){return 9007199254740991});
p("Object.values",function(a){return a?a:function(b){var c=[],d;for(d in b)Ha(b,d)&&c.push(b[d]);return c}});
var x=this||self;function z(a,b,c){a=a.split(".");c=c||x;a[0]in c||"undefined"==typeof c.execScript||c.execScript("var "+a[0]);for(var d;a.length&&(d=a.shift());)a.length||void 0===b?c[d]&&c[d]!==Object.prototype[d]?c=c[d]:c=c[d]={}:c[d]=b}
function A(a,b){a=a.split(".");b=b||x;for(var c=0;c<a.length;c++)if(b=b[a[c]],null==b)return null;return b}
function La(){}
function Ma(a){a.eb=void 0;a.getInstance=function(){return a.eb?a.eb:a.eb=new a}}
function Na(a){var b=typeof a;return"object"!=b?b:a?Array.isArray(a)?"array":b:"null"}
function Oa(a){var b=Na(a);return"array"==b||"object"==b&&"number"==typeof a.length}
function Pa(a){var b=typeof a;return"object"==b&&null!=a||"function"==b}
function Qa(a){return Object.prototype.hasOwnProperty.call(a,Ra)&&a[Ra]||(a[Ra]=++Sa)}
var Ra="closure_uid_"+(1E9*Math.random()>>>0),Sa=0;function Ta(a,b,c){return a.call.apply(a.bind,arguments)}
function Ua(a,b,c){if(!a)throw Error();if(2<arguments.length){var d=Array.prototype.slice.call(arguments,2);return function(){var e=Array.prototype.slice.call(arguments);Array.prototype.unshift.apply(e,d);return a.apply(b,e)}}return function(){return a.apply(b,arguments)}}
function Va(a,b,c){Function.prototype.bind&&-1!=Function.prototype.bind.toString().indexOf("native code")?Va=Ta:Va=Ua;return Va.apply(null,arguments)}
function Wa(a,b){var c=Array.prototype.slice.call(arguments,1);return function(){var d=c.slice();d.push.apply(d,arguments);return a.apply(this,d)}}
function Xa(a,b){z(a,b,void 0)}
function Ya(a,b){function c(){}
c.prototype=b.prototype;a.aa=b.prototype;a.prototype=new c;a.prototype.constructor=a;a.vo=function(d,e,f){for(var g=Array(arguments.length-2),h=2;h<arguments.length;h++)g[h-2]=arguments[h];return b.prototype[e].apply(d,g)}}
function Za(a){return a}
;function bb(a,b){if(Error.captureStackTrace)Error.captureStackTrace(this,bb);else{var c=Error().stack;c&&(this.stack=c)}a&&(this.message=String(a));void 0!==b&&(this.Ob=b)}
Ya(bb,Error);bb.prototype.name="CustomError";function cb(a){a=a.url;var b=/[?&]dsh=1(&|$)/.test(a);this.j=!b&&/[?&]ae=1(&|$)/.test(a);this.l=!b&&/[?&]ae=2(&|$)/.test(a);if((this.h=/[?&]adurl=([^&]*)/.exec(a))&&this.h[1]){try{var c=decodeURIComponent(this.h[1])}catch(d){c=null}this.i=c}}
;function db(a){var b=!1,c;return function(){b||(c=a(),b=!0);return c}}
;var eb=Array.prototype.indexOf?function(a,b){return Array.prototype.indexOf.call(a,b,void 0)}:function(a,b){if("string"===typeof a)return"string"!==typeof b||1!=b.length?-1:a.indexOf(b,0);
for(var c=0;c<a.length;c++)if(c in a&&a[c]===b)return c;return-1},fb=Array.prototype.forEach?function(a,b,c){Array.prototype.forEach.call(a,b,c)}:function(a,b,c){for(var d=a.length,e="string"===typeof a?a.split(""):a,f=0;f<d;f++)f in e&&b.call(c,e[f],f,a)},gb=Array.prototype.filter?function(a,b){return Array.prototype.filter.call(a,b,void 0)}:function(a,b){for(var c=a.length,d=[],e=0,f="string"===typeof a?a.split(""):a,g=0;g<c;g++)if(g in f){var h=f[g];
b.call(void 0,h,g,a)&&(d[e++]=h)}return d},hb=Array.prototype.map?function(a,b){return Array.prototype.map.call(a,b,void 0)}:function(a,b){for(var c=a.length,d=Array(c),e="string"===typeof a?a.split(""):a,f=0;f<c;f++)f in e&&(d[f]=b.call(void 0,e[f],f,a));
return d},ib=Array.prototype.reduce?function(a,b,c){return Array.prototype.reduce.call(a,b,c)}:function(a,b,c){var d=c;
fb(a,function(e,f){d=b.call(void 0,d,e,f,a)});
return d};
function jb(a,b){a:{for(var c=a.length,d="string"===typeof a?a.split(""):a,e=0;e<c;e++)if(e in d&&b.call(void 0,d[e],e,a)){b=e;break a}b=-1}return 0>b?null:"string"===typeof a?a.charAt(b):a[b]}
function kb(a,b){b=eb(a,b);var c;(c=0<=b)&&Array.prototype.splice.call(a,b,1);return c}
function mb(a,b){for(var c=1;c<arguments.length;c++){var d=arguments[c];if(Oa(d)){var e=a.length||0,f=d.length||0;a.length=e+f;for(var g=0;g<f;g++)a[e+g]=d[g]}else a.push(d)}}
;function nb(a,b){for(var c in a)b.call(void 0,a[c],c,a)}
function ob(a){var b=pb,c;for(c in b)if(a.call(void 0,b[c],c,b))return c}
function qb(a){for(var b in a)return!1;return!0}
function rb(a,b){if(null!==a&&b in a)throw Error('The object already contains the key "'+b+'"');a[b]=!0}
function sb(){var a=B("PLAYER_VARS",{});return null!==a&&"privembed"in a?a.privembed:!1}
function tb(a,b){for(var c in a)if(!(c in b)||a[c]!==b[c])return!1;for(var d in b)if(!(d in a))return!1;return!0}
function ub(a){var b={},c;for(c in a)b[c]=a[c];return b}
function vb(a){if(!a||"object"!==typeof a)return a;if("function"===typeof a.clone)return a.clone();if("undefined"!==typeof Map&&a instanceof Map)return new Map(a);if("undefined"!==typeof Set&&a instanceof Set)return new Set(a);var b=Array.isArray(a)?[]:"function"!==typeof ArrayBuffer||"function"!==typeof ArrayBuffer.isView||!ArrayBuffer.isView(a)||a instanceof DataView?{}:new a.constructor(a.length),c;for(c in a)b[c]=vb(a[c]);return b}
var wb="constructor hasOwnProperty isPrototypeOf propertyIsEnumerable toLocaleString toString valueOf".split(" ");function xb(a,b){for(var c,d,e=1;e<arguments.length;e++){d=arguments[e];for(c in d)a[c]=d[c];for(var f=0;f<wb.length;f++)c=wb[f],Object.prototype.hasOwnProperty.call(d,c)&&(a[c]=d[c])}}
;var yb;function zb(){if(void 0===yb){var a=null,b=x.trustedTypes;if(b&&b.createPolicy){try{a=b.createPolicy("goog#html",{createHTML:Za,createScript:Za,createScriptURL:Za})}catch(c){x.console&&x.console.error(c.message)}yb=a}else yb=a}return yb}
;function Ab(a,b){this.j=a===Bb&&b||""}
Ab.prototype.i=!0;Ab.prototype.h=function(){return this.j};
function Cb(a){return new Ab(Bb,a)}
var Bb={};Cb("");var Db={};function Eb(a){this.j=Db===Db?a:"";this.i=!0}
Eb.prototype.h=function(){return this.j.toString()};
Eb.prototype.toString=function(){return this.j.toString()};function Fb(a,b){this.j=b===Gb?a:""}
Fb.prototype.i=!0;Fb.prototype.h=function(){return this.j.toString()};
Fb.prototype.toString=function(){return this.j+""};
function Hb(a){if(a instanceof Fb&&a.constructor===Fb)return a.j;Na(a);return"type_error:TrustedResourceUrl"}
var Gb={};function Ib(a){var b=zb();a=b?b.createScriptURL(a):a;return new Fb(a,Gb)}
;var Jb=String.prototype.trim?function(a){return a.trim()}:function(a){return/^[\s\xa0]*([\s\S]*?)[\s\xa0]*$/.exec(a)[1]};function Kb(a,b){this.j=b===Lb?a:""}
Kb.prototype.i=!0;Kb.prototype.h=function(){return this.j.toString()};
Kb.prototype.toString=function(){return this.j.toString()};
function Mb(a){if(a instanceof Kb&&a.constructor===Kb)return a.j;Na(a);return"type_error:SafeUrl"}
var Nb=/^(?:(?:https?|mailto|ftp):|[^:/?#]*(?:[/?#]|$))/i,Lb={};function Ob(){var a=x.navigator;return a&&(a=a.userAgent)?a:""}
function C(a){return-1!=Ob().indexOf(a)}
;function Pb(){return(C("Chrome")||C("CriOS"))&&!C("Edge")||C("Silk")}
;var Qb={};function Rb(a){this.j=Qb===Qb?a:"";this.i=!0}
Rb.prototype.h=function(){return this.j.toString()};
Rb.prototype.toString=function(){return this.j.toString()};function Sb(a,b){b instanceof Kb||b instanceof Kb||(b="object"==typeof b&&b.i?b.h():String(b),Nb.test(b)||(b="about:invalid#zClosurez"),b=new Kb(b,Lb));a.href=Mb(b)}
function Tb(a,b){a.rel="stylesheet";a.href=Hb(b).toString();(b=Ub('style[nonce],link[rel="stylesheet"][nonce]',a.ownerDocument&&a.ownerDocument.defaultView))&&a.setAttribute("nonce",b)}
function Vb(){return Ub("script[nonce]",void 0)}
var Wb=/^[\w+/_-]+[=]{0,2}$/;function Ub(a,b){b=(b||x).document;return b.querySelector?(a=b.querySelector(a))&&(a=a.nonce||a.getAttribute("nonce"))&&Wb.test(a)?a:"":""}
;function Xb(a){for(var b=0,c=0;c<a.length;++c)b=31*b+a.charCodeAt(c)>>>0;return b}
;var Yb=RegExp("^(?:([^:/?#.]+):)?(?://(?:([^\\\\/?#]*)@)?([^\\\\/?#]*?)(?::([0-9]+))?(?=[\\\\/?#]|$))?([^?#]+)?(?:\\?([^#]*))?(?:#([\\s\\S]*))?$");function Zb(a){return a?decodeURI(a):a}
function $b(a){return Zb(a.match(Yb)[3]||null)}
function ac(a){var b=a.match(Yb);a=b[5];var c=b[6];b=b[7];var d="";a&&(d+=a);c&&(d+="?"+c);b&&(d+="#"+b);return d}
function bc(a,b,c){if(Array.isArray(b))for(var d=0;d<b.length;d++)bc(a,String(b[d]),c);else null!=b&&c.push(a+(""===b?"":"="+encodeURIComponent(String(b))))}
function cc(a){var b=[],c;for(c in a)bc(c,a[c],b);return b.join("&")}
function dc(a,b){b=cc(b);if(b){var c=a.indexOf("#");0>c&&(c=a.length);var d=a.indexOf("?");if(0>d||d>c){d=c;var e=""}else e=a.substring(d+1,c);a=[a.substr(0,d),e,a.substr(c)];c=a[1];a[1]=b?c?c+"&"+b:b:c;b=a[0]+(a[1]?"?"+a[1]:"")+a[2]}else b=a;return b}
var ec=/#|$/;function fc(){return C("iPhone")&&!C("iPod")&&!C("iPad")}
;function gc(a){gc[" "](a);return a}
gc[" "]=La;var hc=C("Opera"),ic=C("Trident")||C("MSIE"),jc=C("Edge"),kc=C("Gecko")&&!(-1!=Ob().toLowerCase().indexOf("webkit")&&!C("Edge"))&&!(C("Trident")||C("MSIE"))&&!C("Edge"),lc=-1!=Ob().toLowerCase().indexOf("webkit")&&!C("Edge"),mc=C("Android");function nc(){var a=x.document;return a?a.documentMode:void 0}
var oc;a:{var pc="",qc=function(){var a=Ob();if(kc)return/rv:([^\);]+)(\)|;)/.exec(a);if(jc)return/Edge\/([\d\.]+)/.exec(a);if(ic)return/\b(?:MSIE|rv)[: ]([^\);]+)(\)|;)/.exec(a);if(lc)return/WebKit\/(\S+)/.exec(a);if(hc)return/(?:Version)[ \/]?(\S+)/.exec(a)}();
qc&&(pc=qc?qc[1]:"");if(ic){var rc=nc();if(null!=rc&&rc>parseFloat(pc)){oc=String(rc);break a}}oc=pc}var sc=oc,tc;if(x.document&&ic){var uc=nc();tc=uc?uc:parseInt(sc,10)||void 0}else tc=void 0;var vc=tc;var wc=fc()||C("iPod"),xc=C("iPad");!C("Android")||Pb();Pb();var yc=C("Safari")&&!(Pb()||C("Coast")||C("Opera")||C("Edge")||C("Edg/")||C("OPR")||C("Firefox")||C("FxiOS")||C("Silk")||C("Android"))&&!(fc()||C("iPad")||C("iPod"));var zc={},Ac=null;
function Bc(a,b){Oa(a);void 0===b&&(b=0);if(!Ac){Ac={};for(var c="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789".split(""),d=["+/=","+/","-_=","-_.","-_"],e=0;5>e;e++){var f=c.concat(d[e].split(""));zc[e]=f;for(var g=0;g<f.length;g++){var h=f[g];void 0===Ac[h]&&(Ac[h]=g)}}}b=zc[b];c=Array(Math.floor(a.length/3));d=b[64]||"";for(e=f=0;f<a.length-2;f+=3){var k=a[f],l=a[f+1];h=a[f+2];g=b[k>>2];k=b[(k&3)<<4|l>>4];l=b[(l&15)<<2|h>>6];h=b[h&63];c[e++]=""+g+k+l+h}g=0;h=d;switch(a.length-
f){case 2:g=a[f+1],h=b[(g&15)<<2]||d;case 1:a=a[f],c[e]=""+b[a>>2]+b[(a&3)<<4|g>>4]+h+d}return c.join("")}
;var Cc="function"===typeof Uint8Array;var Dc="function"===typeof Symbol&&"symbol"===typeof Symbol()?Symbol(void 0):void 0;function Ec(a,b){Object.isFrozen(a)||(Dc?a[Dc]|=b:void 0!==a.h?a.h|=b:Object.defineProperties(a,{h:{value:b,configurable:!0,writable:!0,enumerable:!1}}))}
function Fc(a){var b;Dc?b=a[Dc]:b=a.h;return null==b?0:b}
function Gc(a){Ec(a,1);return a}
function Hc(a){return Array.isArray(a)?!!(Fc(a)&2):!1}
function Ic(a){if(!Array.isArray(a))throw Error("cannot mark non-array as immutable");Ec(a,2)}
;function Jc(a){return null!==a&&"object"===typeof a&&!Array.isArray(a)&&a.constructor===Object}
var Kc,Lc=Object.freeze(Gc([]));function Mc(a){if(Hc(a.G))throw Error("Cannot mutate an immutable Message");}
var Nc="undefined"!=typeof Symbol&&"undefined"!=typeof Symbol.hasInstance;function Oc(a){return{value:a,configurable:!1,writable:!1,enumerable:!1}}
;function Pc(a,b,c){return-1===b?null:b>=a.l?a.i?a.i[b]:void 0:(void 0===c?0:c)&&a.i&&(c=a.i[b],null!=c)?c:a.G[b+a.j]}
function F(a,b,c,d,e){d=void 0===d?!1:d;(void 0===e?0:e)||Mc(a);b<a.l&&!d?a.G[b+a.j]=c:(a.i||(a.i=a.G[a.l+a.j]={}))[b]=c;return a}
function Qc(a,b,c,d){c=void 0===c?!0:c;d=void 0===d?!1:d;var e=Pc(a,b,d);null==e&&(e=Lc);if(Hc(a.G))c&&(Ic(e),Object.freeze(e));else if(e===Lc||Hc(e))e=Gc(e.slice()),F(a,b,e,d);return e}
function Rc(a,b,c,d){Mc(a);(c=Sc(a,c))&&c!==b&&null!=d&&(a.h&&c in a.h&&(a.h[c]=void 0),F(a,c,void 0));return F(a,b,d)}
function Sc(a,b){for(var c=0,d=0;d<b.length;d++){var e=b[d];null!=Pc(a,e)&&(0!==c&&F(a,c,void 0,!1,!0),c=e)}return c}
function Tc(a,b,c,d,e){if(-1===c)return null;a.h||(a.h={});var f=a.h[c];if(f)return f;e=Pc(a,c,void 0===e?!1:e);if(null==e&&!d)return f;b=new b(e);Hc(a.G)&&Ic(b.G);return a.h[c]=b}
function Uc(a,b,c,d){d=void 0===d?!1:d;a.h||(a.h={});var e=Hc(a.G),f=a.h[c];if(!f){d=Qc(a,c,!0,d);f=[];e=e||Hc(d);for(var g=0;g<d.length;g++)f[g]=new b(d[g]),e&&Ic(f[g].G);e&&(Ic(f),Object.freeze(f));a.h[c]=f}return f}
function G(a,b,c,d){d=void 0===d?!1:d;Mc(a);a.h||(a.h={});var e=c?c.G:c;a.h[b]=c;return F(a,b,e,d)}
function Vc(a,b,c){var d=Wc;Mc(a);a.h||(a.h={});var e=c?c.G:c;a.h[b]=c;Rc(a,b,d,e)}
function Xc(a,b,c,d){var e=void 0===e?!1:e;Mc(a);e=Uc(a,c,b,e);c=d?d:new c;a=Qc(a,b);e.push(c);a.push(c.G)}
;function Yc(a){switch(typeof a){case "number":return isFinite(a)?a:String(a);case "object":if(a&&!Array.isArray(a)&&Cc&&null!=a&&a instanceof Uint8Array)return Bc(a)}return a}
;function Zc(a,b){b=void 0===b?$c:b;return ad(a,b)}
function bd(a,b){if(null!=a){if(Array.isArray(a))a=ad(a,b);else if(Jc(a)){var c={},d;for(d in a)c[d]=bd(a[d],b);a=c}else a=b(a);return a}}
function ad(a,b){for(var c=a.slice(),d=0;d<c.length;d++)c[d]=bd(c[d],b);Array.isArray(a)&&Fc(a)&1&&Gc(c);return c}
function cd(a){if(a&&"object"==typeof a&&a.toJSON)return a.toJSON();a=Yc(a);return Array.isArray(a)?Zc(a,cd):a}
function $c(a){return Cc&&null!=a&&a instanceof Uint8Array?new Uint8Array(a):a}
;var dd;function ed(a,b){dd=b;a=new a(b);dd=null;return a}
;function fd(a,b,c){var d=dd;dd=null;a||(a=d);d=this.constructor.i;a||(a=d?[d]:[]);this.j=(d?0:-1)-(this.constructor.h||0);this.h=void 0;this.G=a;a:{d=this.G.length;a=d-1;if(d&&(d=this.G[a],Jc(d))){this.l=a-this.j;this.i=d;break a}void 0!==b&&-1<b?(this.l=Math.max(b,a+1-this.j),this.i=void 0):this.l=Number.MAX_VALUE}if(c)for(b=0;b<c.length;b++)if(a=c[b],a<this.l)a+=this.j,(d=this.G[a])?Array.isArray(d)&&Gc(d):this.G[a]=Lc;else{d=this.i||(this.i=this.G[this.l+this.j]={});var e=d[a];e?Array.isArray(e)&&
Gc(e):d[a]=Lc}}
fd.prototype.toJSON=function(){var a=this.G;return Kc?a:Zc(a,cd)};
function gd(a){Kc=!0;try{return JSON.stringify(a.toJSON(),hd)}finally{Kc=!1}}
fd.prototype.clone=function(){var a=this.constructor,b=Zc(this.G);a=ed(a,b);id(a,this);return a};
fd.prototype.toString=function(){return this.G.toString()};
function hd(a,b){return Yc(b)}
function id(a,b){b.o&&(a.o=b.o.slice());var c=b.h;if(c){b=b.i;for(var d in c){var e=c[d];if(e){var f=!(!b||!b[d]),g=+d;if(Array.isArray(e)){if(e.length)for(f=Uc(a,e[0].constructor,g,f),g=0;g<Math.min(f.length,e.length);g++)id(f[g],e[g])}else(f=Tc(a,e.constructor,g,void 0,f))&&id(f,e)}}}}
;function jd(){fd.apply(this,arguments)}
r(jd,fd);function kd(){var a={};Object.defineProperties(jd,(a[Symbol.hasInstance]=Oc(function(){throw Error("Cannot perform instanceof checks for MutableMessage");}),a))}
Nc&&kd();function ld(a,b){var c=this.h;if(this.isRepeated){var d=!0;d=void 0===d?!1:d;Mc(a);if(b){var e=Gc([]);for(var f=0;f<b.length;f++)e[f]=b[f].G;a.h||(a.h={});a.h[c]=b}else a.h&&(a.h[c]=void 0),e=Lc;a=F(a,c,e,d)}else a=G(a,c,b,!0);return a}
;function md(a){x.setTimeout(function(){throw a;},0)}
;function H(){jd.apply(this,arguments)}
r(H,jd);function nd(){var a={};Object.defineProperties(H,(a[Symbol.hasInstance]=Oc(Object[Symbol.hasInstance]),a))}
Nc&&nd();function od(){var a=this;this.promise=new Promise(function(b,c){a.resolve=b;a.reject=c})}
;function pd(a){this.i=!1;var b=a.program;a=a.globalName;var c=new od;this.j=c.promise;this.l=q((0,x[a].a)(b,function(d,e){Promise.resolve().then(function(){c.resolve({Nb:d,vc:e})})},!0)).next().value;
this.uc=c.promise.then(function(){})}
pd.prototype.snapshot=function(a){if(this.i)throw Error("Already disposed");return this.j.then(function(b){var c=b.Nb;return new Promise(function(d){c(function(e){d(e)},[a.qb,
a.wc])})})};
pd.prototype.dispose=function(){this.i=!0;this.j.then(function(a){(a=a.vc)&&a()})};
pd.prototype.h=function(){return this.i};var qd=window;Cb("csi.gstatic.com");Cb("googleads.g.doubleclick.net");Cb("partner.googleadservices.com");Cb("pubads.g.doubleclick.net");Cb("securepubads.g.doubleclick.net");Cb("tpc.googlesyndication.com");/*

 SPDX-License-Identifier: Apache-2.0
*/
var rd={};function sd(){}
function td(a){this.h=a}
r(td,sd);td.prototype.toString=function(){return this.h};
var ud=new td("about:invalid#zTSz",rd);function vd(a){if(a instanceof sd)if(a instanceof td)a=a.h;else throw Error("");else a=Mb(a);return a}
;function wd(a,b){a.src=Hb(b);var c;b=(a.ownerDocument&&a.ownerDocument.defaultView||window).document;var d=null===(c=b.querySelector)||void 0===c?void 0:c.call(b,"script[nonce]");(c=d?d.nonce||d.getAttribute("nonce")||"":"")&&a.setAttribute("nonce",c)}
;function xd(a,b){this.x=void 0!==a?a:0;this.y=void 0!==b?b:0}
m=xd.prototype;m.clone=function(){return new xd(this.x,this.y)};
m.equals=function(a){return a instanceof xd&&(this==a?!0:this&&a?this.x==a.x&&this.y==a.y:!1)};
m.ceil=function(){this.x=Math.ceil(this.x);this.y=Math.ceil(this.y);return this};
m.floor=function(){this.x=Math.floor(this.x);this.y=Math.floor(this.y);return this};
m.round=function(){this.x=Math.round(this.x);this.y=Math.round(this.y);return this};
m.scale=function(a,b){this.x*=a;this.y*="number"===typeof b?b:a;return this};function yd(a,b){this.width=a;this.height=b}
m=yd.prototype;m.clone=function(){return new yd(this.width,this.height)};
m.aspectRatio=function(){return this.width/this.height};
m.isEmpty=function(){return!(this.width*this.height)};
m.ceil=function(){this.width=Math.ceil(this.width);this.height=Math.ceil(this.height);return this};
m.floor=function(){this.width=Math.floor(this.width);this.height=Math.floor(this.height);return this};
m.round=function(){this.width=Math.round(this.width);this.height=Math.round(this.height);return this};
m.scale=function(a,b){this.width*=a;this.height*="number"===typeof b?b:a;return this};function zd(a){var b=document;return"string"===typeof a?b.getElementById(a):a}
function Ad(a){var b=document;a=String(a);"application/xhtml+xml"===b.contentType&&(a=a.toLowerCase());return b.createElement(a)}
function Bd(a,b){for(var c=0;a;){if(b(a))return a;a=a.parentNode;c++}return null}
;function Dd(a){var b=Ed;if(b)for(var c in b)Object.prototype.hasOwnProperty.call(b,c)&&a(b[c],c,b)}
function Fd(){var a=[];Dd(function(b){a.push(b)});
return a}
var Ed={Lc:"allow-forms",Mc:"allow-modals",Nc:"allow-orientation-lock",Oc:"allow-pointer-lock",Pc:"allow-popups",Qc:"allow-popups-to-escape-sandbox",Rc:"allow-presentation",Sc:"allow-same-origin",Tc:"allow-scripts",Uc:"allow-top-navigation",Vc:"allow-top-navigation-by-user-activation"},Gd=db(function(){return Fd()});
function Hd(){var a=Id(),b={};fb(Gd(),function(c){a.sandbox&&a.sandbox.supports&&a.sandbox.supports(c)&&(b[c]=!0)});
return b}
function Id(){var a=void 0===a?document:a;return a.createElement("iframe")}
;function Jd(a){this.isValid=a}
function Kd(a){return new Jd(function(b){return b.substr(0,a.length+1).toLowerCase()===a+":"})}
var Ld=[Kd("data"),Kd("http"),Kd("https"),Kd("mailto"),Kd("ftp"),new Jd(function(a){return/^[^:]*([/?#]|$)/.test(a)})];function Md(a){"number"==typeof a&&(a=Math.round(a)+"px");return a}
;var Nd=(new Date).getTime();function Od(a){if(!a)return"";if(/^about:(?:blank|srcdoc)$/.test(a))return window.origin||"";a=a.split("#")[0].split("?")[0];a=a.toLowerCase();0==a.indexOf("//")&&(a=window.location.protocol+a);/^[\w\-]*:\/\//.test(a)||(a=window.location.href);var b=a.substring(a.indexOf("://")+3),c=b.indexOf("/");-1!=c&&(b=b.substring(0,c));c=a.substring(0,a.indexOf("://"));if(!c)throw Error("URI is missing protocol: "+a);if("http"!==c&&"https"!==c&&"chrome-extension"!==c&&"moz-extension"!==c&&"file"!==c&&"android-app"!==
c&&"chrome-search"!==c&&"chrome-untrusted"!==c&&"chrome"!==c&&"app"!==c&&"devtools"!==c)throw Error("Invalid URI scheme in origin: "+c);a="";var d=b.indexOf(":");if(-1!=d){var e=b.substring(d+1);b=b.substring(0,d);if("http"===c&&"80"!==e||"https"===c&&"443"!==e)a=":"+e}return c+"://"+b+a}
;function Pd(){function a(){e[0]=1732584193;e[1]=4023233417;e[2]=2562383102;e[3]=271733878;e[4]=3285377520;n=l=0}
function b(t){for(var y=g,u=0;64>u;u+=4)y[u/4]=t[u]<<24|t[u+1]<<16|t[u+2]<<8|t[u+3];for(u=16;80>u;u++)t=y[u-3]^y[u-8]^y[u-14]^y[u-16],y[u]=(t<<1|t>>>31)&4294967295;t=e[0];var D=e[1],E=e[2],L=e[3],P=e[4];for(u=0;80>u;u++){if(40>u)if(20>u){var R=L^D&(E^L);var V=1518500249}else R=D^E^L,V=1859775393;else 60>u?(R=D&E|L&(D|E),V=2400959708):(R=D^E^L,V=3395469782);R=((t<<5|t>>>27)&4294967295)+R+P+V+y[u]&4294967295;P=L;L=E;E=(D<<30|D>>>2)&4294967295;D=t;t=R}e[0]=e[0]+t&4294967295;e[1]=e[1]+D&4294967295;e[2]=
e[2]+E&4294967295;e[3]=e[3]+L&4294967295;e[4]=e[4]+P&4294967295}
function c(t,y){if("string"===typeof t){t=unescape(encodeURIComponent(t));for(var u=[],D=0,E=t.length;D<E;++D)u.push(t.charCodeAt(D));t=u}y||(y=t.length);u=0;if(0==l)for(;u+64<y;)b(t.slice(u,u+64)),u+=64,n+=64;for(;u<y;)if(f[l++]=t[u++],n++,64==l)for(l=0,b(f);u+64<y;)b(t.slice(u,u+64)),u+=64,n+=64}
function d(){var t=[],y=8*n;56>l?c(h,56-l):c(h,64-(l-56));for(var u=63;56<=u;u--)f[u]=y&255,y>>>=8;b(f);for(u=y=0;5>u;u++)for(var D=24;0<=D;D-=8)t[y++]=e[u]>>D&255;return t}
for(var e=[],f=[],g=[],h=[128],k=1;64>k;++k)h[k]=0;var l,n;a();return{reset:a,update:c,digest:d,Rb:function(){for(var t=d(),y="",u=0;u<t.length;u++)y+="0123456789ABCDEF".charAt(Math.floor(t[u]/16))+"0123456789ABCDEF".charAt(t[u]%16);return y}}}
;function Qd(a,b,c){var d=String(x.location.href);return d&&a&&b?[b,Rd(Od(d),a,c||null)].join(" "):null}
function Rd(a,b,c){var d=[],e=[];if(1==(Array.isArray(c)?2:1))return e=[b,a],fb(d,function(h){e.push(h)}),Sd(e.join(" "));
var f=[],g=[];fb(c,function(h){g.push(h.key);f.push(h.value)});
c=Math.floor((new Date).getTime()/1E3);e=0==f.length?[c,b,a]:[f.join(":"),c,b,a];fb(d,function(h){e.push(h)});
a=Sd(e.join(" "));a=[c,a];0==g.length||a.push(g.join(""));return a.join("_")}
function Sd(a){var b=Pd();b.update(a);return b.Rb().toLowerCase()}
;var Td={};function Ud(a){this.h=a||{cookie:""}}
m=Ud.prototype;m.isEnabled=function(){if(!x.navigator.cookieEnabled)return!1;if(!this.isEmpty())return!0;this.set("TESTCOOKIESENABLED","1",{Pa:60});if("1"!==this.get("TESTCOOKIESENABLED"))return!1;this.remove("TESTCOOKIESENABLED");return!0};
m.set=function(a,b,c){var d=!1;if("object"===typeof c){var e=c.Ho;d=c.secure||!1;var f=c.domain||void 0;var g=c.path||void 0;var h=c.Pa}if(/[;=\s]/.test(a))throw Error('Invalid cookie name "'+a+'"');if(/[;\r\n]/.test(b))throw Error('Invalid cookie value "'+b+'"');void 0===h&&(h=-1);c=f?";domain="+f:"";g=g?";path="+g:"";d=d?";secure":"";h=0>h?"":0==h?";expires="+(new Date(1970,1,1)).toUTCString():";expires="+(new Date(Date.now()+1E3*h)).toUTCString();this.h.cookie=a+"="+b+c+g+h+d+(null!=e?";samesite="+
e:"")};
m.get=function(a,b){for(var c=a+"=",d=(this.h.cookie||"").split(";"),e=0,f;e<d.length;e++){f=Jb(d[e]);if(0==f.lastIndexOf(c,0))return f.substr(c.length);if(f==a)return""}return b};
m.remove=function(a,b,c){var d=void 0!==this.get(a);this.set(a,"",{Pa:0,path:b,domain:c});return d};
m.Za=function(){return Vd(this).keys};
m.isEmpty=function(){return!this.h.cookie};
m.clear=function(){for(var a=Vd(this).keys,b=a.length-1;0<=b;b--)this.remove(a[b])};
function Vd(a){a=(a.h.cookie||"").split(";");for(var b=[],c=[],d,e,f=0;f<a.length;f++)e=Jb(a[f]),d=e.indexOf("="),-1==d?(b.push(""),c.push(e)):(b.push(e.substring(0,d)),c.push(e.substring(d+1)));return{keys:b,values:c}}
var Wd=new Ud("undefined"==typeof document?null:document);function Xd(a){return!!Td.FPA_SAMESITE_PHASE2_MOD||!(void 0===a||!a)}
function Yd(a){a=void 0===a?!1:a;var b=x.__SAPISID||x.__APISID||x.__3PSAPISID||x.__OVERRIDE_SID;Xd(a)&&(b=b||x.__1PSAPISID);if(b)return!0;var c=new Ud(document);b=c.get("SAPISID")||c.get("APISID")||c.get("__Secure-3PAPISID")||c.get("SID");Xd(a)&&(b=b||c.get("__Secure-1PAPISID"));return!!b}
function Zd(a,b,c,d){(a=x[a])||(a=(new Ud(document)).get(b));return a?Qd(a,c,d):null}
function $d(a){var b=void 0===b?!1:b;var c=Od(String(x.location.href)),d=[];if(Yd(b)){c=0==c.indexOf("https:")||0==c.indexOf("chrome-extension:")||0==c.indexOf("moz-extension:");var e=c?x.__SAPISID:x.__APISID;e||(e=new Ud(document),e=e.get(c?"SAPISID":"APISID")||e.get("__Secure-3PAPISID"));(e=e?Qd(e,c?"SAPISIDHASH":"APISIDHASH",a):null)&&d.push(e);c&&Xd(b)&&((b=Zd("__1PSAPISID","__Secure-1PAPISID","SAPISID1PHASH",a))&&d.push(b),(a=Zd("__3PSAPISID","__Secure-3PAPISID","SAPISID3PHASH",a))&&d.push(a))}return 0==
d.length?null:d.join(" ")}
;function ae(a){Nc&&Object.defineProperty(a,Symbol.hasInstance,Oc(Object[Symbol.hasInstance]))}
;function be(a){a&&"function"==typeof a.dispose&&a.dispose()}
;function ce(a){for(var b=0,c=arguments.length;b<c;++b){var d=arguments[b];Oa(d)?ce.apply(null,d):be(d)}}
;function I(){this.D=this.D;this.o=this.o}
I.prototype.D=!1;I.prototype.h=function(){return this.D};
I.prototype.dispose=function(){this.D||(this.D=!0,this.I())};
function de(a,b){a.D?b():(a.o||(a.o=[]),a.o.push(b))}
I.prototype.I=function(){if(this.o)for(;this.o.length;)this.o.shift()()};function ee(a,b){this.type=a;this.h=this.target=b;this.defaultPrevented=this.j=!1}
ee.prototype.stopPropagation=function(){this.j=!0};
ee.prototype.preventDefault=function(){this.defaultPrevented=!0};function fe(a){var b=A("window.location.href");null==a&&(a='Unknown Error of type "null/undefined"');if("string"===typeof a)return{message:a,name:"Unknown error",lineNumber:"Not available",fileName:b,stack:"Not available"};var c=!1;try{var d=a.lineNumber||a.line||"Not available"}catch(g){d="Not available",c=!0}try{var e=a.fileName||a.filename||a.sourceURL||x.$googDebugFname||b}catch(g){e="Not available",c=!0}b=ge(a);if(!(!c&&a.lineNumber&&a.fileName&&a.stack&&a.message&&a.name)){c=a.message;if(null==
c){if(a.constructor&&a.constructor instanceof Function){if(a.constructor.name)c=a.constructor.name;else if(c=a.constructor,he[c])c=he[c];else{c=String(c);if(!he[c]){var f=/function\s+([^\(]+)/m.exec(c);he[c]=f?f[1]:"[Anonymous]"}c=he[c]}c='Unknown Error of type "'+c+'"'}else c="Unknown Error of unknown type";"function"===typeof a.toString&&Object.prototype.toString!==a.toString&&(c+=": "+a.toString())}return{message:c,name:a.name||"UnknownError",lineNumber:d,fileName:e,stack:b||"Not available"}}a.stack=
b;return{message:a.message,name:a.name,lineNumber:a.lineNumber,fileName:a.fileName,stack:a.stack}}
function ge(a,b){b||(b={});b[ie(a)]=!0;var c=a.stack||"";(a=a.Ob)&&!b[ie(a)]&&(c+="\nCaused by: ",a.stack&&0==a.stack.indexOf(a.toString())||(c+="string"===typeof a?a:a.message+"\n"),c+=ge(a,b));return c}
function ie(a){var b="";"function"===typeof a.toString&&(b=""+a);return b+a.stack}
var he={};var je=function(){if(!x.addEventListener||!Object.defineProperty)return!1;var a=!1,b=Object.defineProperty({},"passive",{get:function(){a=!0}});
try{x.addEventListener("test",La,b),x.removeEventListener("test",La,b)}catch(c){}return a}();function ke(a,b){ee.call(this,a?a.type:"");this.relatedTarget=this.h=this.target=null;this.button=this.screenY=this.screenX=this.clientY=this.clientX=0;this.key="";this.charCode=this.keyCode=0;this.metaKey=this.shiftKey=this.altKey=this.ctrlKey=!1;this.state=null;this.pointerId=0;this.pointerType="";this.i=null;a&&this.init(a,b)}
Ya(ke,ee);var le={2:"touch",3:"pen",4:"mouse"};
ke.prototype.init=function(a,b){var c=this.type=a.type,d=a.changedTouches&&a.changedTouches.length?a.changedTouches[0]:null;this.target=a.target||a.srcElement;this.h=b;if(b=a.relatedTarget){if(kc){a:{try{gc(b.nodeName);var e=!0;break a}catch(f){}e=!1}e||(b=null)}}else"mouseover"==c?b=a.fromElement:"mouseout"==c&&(b=a.toElement);this.relatedTarget=b;d?(this.clientX=void 0!==d.clientX?d.clientX:d.pageX,this.clientY=void 0!==d.clientY?d.clientY:d.pageY,this.screenX=d.screenX||0,this.screenY=d.screenY||
0):(this.clientX=void 0!==a.clientX?a.clientX:a.pageX,this.clientY=void 0!==a.clientY?a.clientY:a.pageY,this.screenX=a.screenX||0,this.screenY=a.screenY||0);this.button=a.button;this.keyCode=a.keyCode||0;this.key=a.key||"";this.charCode=a.charCode||("keypress"==c?a.keyCode:0);this.ctrlKey=a.ctrlKey;this.altKey=a.altKey;this.shiftKey=a.shiftKey;this.metaKey=a.metaKey;this.pointerId=a.pointerId||0;this.pointerType="string"===typeof a.pointerType?a.pointerType:le[a.pointerType]||"";this.state=a.state;
this.i=a;a.defaultPrevented&&ke.aa.preventDefault.call(this)};
ke.prototype.stopPropagation=function(){ke.aa.stopPropagation.call(this);this.i.stopPropagation?this.i.stopPropagation():this.i.cancelBubble=!0};
ke.prototype.preventDefault=function(){ke.aa.preventDefault.call(this);var a=this.i;a.preventDefault?a.preventDefault():a.returnValue=!1};var me="closure_listenable_"+(1E6*Math.random()|0);var ne=0;function oe(a,b,c,d,e){this.listener=a;this.proxy=null;this.src=b;this.type=c;this.capture=!!d;this.Ma=e;this.key=++ne;this.za=this.Ia=!1}
function pe(a){a.za=!0;a.listener=null;a.proxy=null;a.src=null;a.Ma=null}
;function qe(a){this.src=a;this.listeners={};this.h=0}
qe.prototype.add=function(a,b,c,d,e){var f=a.toString();a=this.listeners[f];a||(a=this.listeners[f]=[],this.h++);var g=re(a,b,d,e);-1<g?(b=a[g],c||(b.Ia=!1)):(b=new oe(b,this.src,f,!!d,e),b.Ia=c,a.push(b));return b};
qe.prototype.remove=function(a,b,c,d){a=a.toString();if(!(a in this.listeners))return!1;var e=this.listeners[a];b=re(e,b,c,d);return-1<b?(pe(e[b]),Array.prototype.splice.call(e,b,1),0==e.length&&(delete this.listeners[a],this.h--),!0):!1};
function se(a,b){var c=b.type;c in a.listeners&&kb(a.listeners[c],b)&&(pe(b),0==a.listeners[c].length&&(delete a.listeners[c],a.h--))}
function re(a,b,c,d){for(var e=0;e<a.length;++e){var f=a[e];if(!f.za&&f.listener==b&&f.capture==!!c&&f.Ma==d)return e}return-1}
;var te="closure_lm_"+(1E6*Math.random()|0),ue={},ve=0;function we(a,b,c,d,e){if(d&&d.once)xe(a,b,c,d,e);else if(Array.isArray(b))for(var f=0;f<b.length;f++)we(a,b[f],c,d,e);else c=ye(c),a&&a[me]?a.W(b,c,Pa(d)?!!d.capture:!!d,e):ze(a,b,c,!1,d,e)}
function ze(a,b,c,d,e,f){if(!b)throw Error("Invalid event type");var g=Pa(e)?!!e.capture:!!e,h=Ae(a);h||(a[te]=h=new qe(a));c=h.add(b,c,d,g,f);if(!c.proxy){d=Be();c.proxy=d;d.src=a;d.listener=c;if(a.addEventListener)je||(e=g),void 0===e&&(e=!1),a.addEventListener(b.toString(),d,e);else if(a.attachEvent)a.attachEvent(Ce(b.toString()),d);else if(a.addListener&&a.removeListener)a.addListener(d);else throw Error("addEventListener and attachEvent are unavailable.");ve++}}
function Be(){function a(c){return b.call(a.src,a.listener,c)}
var b=De;return a}
function xe(a,b,c,d,e){if(Array.isArray(b))for(var f=0;f<b.length;f++)xe(a,b[f],c,d,e);else c=ye(c),a&&a[me]?a.i.add(String(b),c,!0,Pa(d)?!!d.capture:!!d,e):ze(a,b,c,!0,d,e)}
function Ee(a,b,c,d,e){if(Array.isArray(b))for(var f=0;f<b.length;f++)Ee(a,b[f],c,d,e);else(d=Pa(d)?!!d.capture:!!d,c=ye(c),a&&a[me])?a.i.remove(String(b),c,d,e):a&&(a=Ae(a))&&(b=a.listeners[b.toString()],a=-1,b&&(a=re(b,c,d,e)),(c=-1<a?b[a]:null)&&Fe(c))}
function Fe(a){if("number"!==typeof a&&a&&!a.za){var b=a.src;if(b&&b[me])se(b.i,a);else{var c=a.type,d=a.proxy;b.removeEventListener?b.removeEventListener(c,d,a.capture):b.detachEvent?b.detachEvent(Ce(c),d):b.addListener&&b.removeListener&&b.removeListener(d);ve--;(c=Ae(b))?(se(c,a),0==c.h&&(c.src=null,b[te]=null)):pe(a)}}}
function Ce(a){return a in ue?ue[a]:ue[a]="on"+a}
function De(a,b){if(a.za)a=!0;else{b=new ke(b,this);var c=a.listener,d=a.Ma||a.src;a.Ia&&Fe(a);a=c.call(d,b)}return a}
function Ae(a){a=a[te];return a instanceof qe?a:null}
var Ge="__closure_events_fn_"+(1E9*Math.random()>>>0);function ye(a){if("function"===typeof a)return a;a[Ge]||(a[Ge]=function(b){return a.handleEvent(b)});
return a[Ge]}
;function He(){I.call(this);this.i=new qe(this);this.Z=this;this.L=null}
Ya(He,I);He.prototype[me]=!0;He.prototype.addEventListener=function(a,b,c,d){we(this,a,b,c,d)};
He.prototype.removeEventListener=function(a,b,c,d){Ee(this,a,b,c,d)};
function Ie(a,b){var c=a.L;if(c){var d=[];for(var e=1;c;c=c.L)d.push(c),++e}a=a.Z;c=b.type||b;"string"===typeof b?b=new ee(b,a):b instanceof ee?b.target=b.target||a:(e=b,b=new ee(c,a),xb(b,e));e=!0;if(d)for(var f=d.length-1;!b.j&&0<=f;f--){var g=b.h=d[f];e=Je(g,c,!0,b)&&e}b.j||(g=b.h=a,e=Je(g,c,!0,b)&&e,b.j||(e=Je(g,c,!1,b)&&e));if(d)for(f=0;!b.j&&f<d.length;f++)g=b.h=d[f],e=Je(g,c,!1,b)&&e}
He.prototype.I=function(){He.aa.I.call(this);if(this.i){var a=this.i,b=0,c;for(c in a.listeners){for(var d=a.listeners[c],e=0;e<d.length;e++)++b,pe(d[e]);delete a.listeners[c];a.h--}}this.L=null};
He.prototype.W=function(a,b,c,d){return this.i.add(String(a),b,!1,c,d)};
function Je(a,b,c,d){b=a.i.listeners[String(b)];if(!b)return!0;b=b.concat();for(var e=!0,f=0;f<b.length;++f){var g=b[f];if(g&&!g.za&&g.capture==c){var h=g.listener,k=g.Ma||g.src;g.Ia&&se(a.i,g);e=!1!==h.call(k,d)&&e}}return e&&!d.defaultPrevented}
;function Ke(a){var b,c;He.call(this);var d=this;this.A=this.l=0;this.V=null!==a&&void 0!==a?a:{N:function(e,f){return setTimeout(e,f)},
U:clearTimeout};this.j=null!==(c=null===(b=window.navigator)||void 0===b?void 0:b.onLine)&&void 0!==c?c:!0;this.m=function(){return w(function(e){return v(e,Le(d),0)})};
window.addEventListener("offline",this.m);window.addEventListener("online",this.m);this.A||Me(this)}
r(Ke,He);Ke.prototype.dispose=function(){window.removeEventListener("offline",this.m);window.removeEventListener("online",this.m);this.V.U(this.A);delete Ke.h};
Ke.prototype.H=function(){return this.j};
function Me(a){a.A=a.V.N(function(){var b;return w(function(c){if(1==c.h)return a.j?(null===(b=window.navigator)||void 0===b?0:b.onLine)?c.s(3):v(c,Le(a),3):v(c,Le(a),3);Me(a);c.h=0})},3E4)}
function Le(a,b){return a.u?a.u:a.u=new Promise(function(c){var d,e,f;return w(function(g){switch(g.h){case 1:return d=window.AbortController?new window.AbortController:void 0,e=null===d||void 0===d?void 0:d.signal,f=!1,ua(g,2,3),d&&(a.l=a.V.N(function(){d.abort()},b||2E4)),v(g,fetch("/generate_204",{method:"HEAD",
signal:e}),5);case 5:f=!0;case 3:xa(g);a.u=void 0;a.l&&(a.V.U(a.l),a.l=0);f!==a.j&&(a.j=f,a.j?Ie(a,"networkstatus-online"):Ie(a,"networkstatus-offline"));c(f);ya(g);break;case 2:wa(g),f=!1,g.s(3)}})})}
;var Ne={co:"WEB_DISPLAY_MODE_UNKNOWN",Yn:"WEB_DISPLAY_MODE_BROWSER",ao:"WEB_DISPLAY_MODE_MINIMAL_UI",bo:"WEB_DISPLAY_MODE_STANDALONE",Zn:"WEB_DISPLAY_MODE_FULLSCREEN"};function Oe(){this.data_=[];this.h=-1}
Oe.prototype.set=function(a,b){b=void 0===b?!0:b;0<=a&&52>a&&0===a%1&&this.data_[a]!=b&&(this.data_[a]=b,this.h=-1)};
Oe.prototype.get=function(a){return!!this.data_[a]};
function Pe(a){-1==a.h&&(a.h=ib(a.data_,function(b,c,d){return c?b+Math.pow(2,d):b},0));
return a.h}
;function Qe(a,b){this.j=a;this.l=b;this.i=0;this.h=null}
Qe.prototype.get=function(){if(0<this.i){this.i--;var a=this.h;this.h=a.next;a.next=null}else a=this.j();return a};
function Re(a,b){a.l(b);100>a.i&&(a.i++,b.next=a.h,a.h=b)}
;var Se;function Te(){var a=x.MessageChannel;"undefined"===typeof a&&"undefined"!==typeof window&&window.postMessage&&window.addEventListener&&!C("Presto")&&(a=function(){var e=Ad("IFRAME");e.style.display="none";document.documentElement.appendChild(e);var f=e.contentWindow;e=f.document;e.open();e.close();var g="callImmediate"+Math.random(),h="file:"==f.location.protocol?"*":f.location.protocol+"//"+f.location.host;e=Va(function(k){if(("*"==h||k.origin==h)&&k.data==g)this.port1.onmessage()},this);
f.addEventListener("message",e,!1);this.port1={};this.port2={postMessage:function(){f.postMessage(g,h)}}});
if("undefined"!==typeof a&&!C("Trident")&&!C("MSIE")){var b=new a,c={},d=c;b.port1.onmessage=function(){if(void 0!==c.next){c=c.next;var e=c.ob;c.ob=null;e()}};
return function(e){d.next={ob:e};d=d.next;b.port2.postMessage(0)}}return function(e){x.setTimeout(e,0)}}
;function Ue(){this.i=this.h=null}
Ue.prototype.add=function(a,b){var c=Ve.get();c.set(a,b);this.i?this.i.next=c:this.h=c;this.i=c};
Ue.prototype.remove=function(){var a=null;this.h&&(a=this.h,this.h=this.h.next,this.h||(this.i=null),a.next=null);return a};
var Ve=new Qe(function(){return new We},function(a){return a.reset()});
function We(){this.next=this.scope=this.h=null}
We.prototype.set=function(a,b){this.h=a;this.scope=b;this.next=null};
We.prototype.reset=function(){this.next=this.scope=this.h=null};function Xe(a,b){Ye||Ze();$e||(Ye(),$e=!0);af.add(a,b)}
var Ye;function Ze(){if(x.Promise&&x.Promise.resolve){var a=x.Promise.resolve(void 0);Ye=function(){a.then(bf)}}else Ye=function(){var b=bf;
"function"!==typeof x.setImmediate||x.Window&&x.Window.prototype&&!C("Edge")&&x.Window.prototype.setImmediate==x.setImmediate?(Se||(Se=Te()),Se(b)):x.setImmediate(b)}}
var $e=!1,af=new Ue;function bf(){for(var a;a=af.remove();){try{a.h.call(a.scope)}catch(b){md(b)}Re(Ve,a)}$e=!1}
;function cf(a,b){this.h=a[x.Symbol.iterator]();this.i=b;this.j=0}
cf.prototype[Symbol.iterator]=function(){return this};
cf.prototype.next=function(){var a=this.h.next();return{value:a.done?void 0:this.i.call(void 0,a.value,this.j++),done:a.done}};
function df(a,b){return new cf(a,b)}
;function ef(){this.blockSize=-1}
;function ff(){this.blockSize=-1;this.blockSize=64;this.h=[];this.o=[];this.m=[];this.j=[];this.j[0]=128;for(var a=1;a<this.blockSize;++a)this.j[a]=0;this.l=this.i=0;this.reset()}
Ya(ff,ef);ff.prototype.reset=function(){this.h[0]=1732584193;this.h[1]=4023233417;this.h[2]=2562383102;this.h[3]=271733878;this.h[4]=3285377520;this.l=this.i=0};
function gf(a,b,c){c||(c=0);var d=a.m;if("string"===typeof b)for(var e=0;16>e;e++)d[e]=b.charCodeAt(c)<<24|b.charCodeAt(c+1)<<16|b.charCodeAt(c+2)<<8|b.charCodeAt(c+3),c+=4;else for(e=0;16>e;e++)d[e]=b[c]<<24|b[c+1]<<16|b[c+2]<<8|b[c+3],c+=4;for(e=16;80>e;e++){var f=d[e-3]^d[e-8]^d[e-14]^d[e-16];d[e]=(f<<1|f>>>31)&4294967295}b=a.h[0];c=a.h[1];var g=a.h[2],h=a.h[3],k=a.h[4];for(e=0;80>e;e++){if(40>e)if(20>e){f=h^c&(g^h);var l=1518500249}else f=c^g^h,l=1859775393;else 60>e?(f=c&g|h&(c|g),l=2400959708):
(f=c^g^h,l=3395469782);f=(b<<5|b>>>27)+f+k+l+d[e]&4294967295;k=h;h=g;g=(c<<30|c>>>2)&4294967295;c=b;b=f}a.h[0]=a.h[0]+b&4294967295;a.h[1]=a.h[1]+c&4294967295;a.h[2]=a.h[2]+g&4294967295;a.h[3]=a.h[3]+h&4294967295;a.h[4]=a.h[4]+k&4294967295}
ff.prototype.update=function(a,b){if(null!=a){void 0===b&&(b=a.length);for(var c=b-this.blockSize,d=0,e=this.o,f=this.i;d<b;){if(0==f)for(;d<=c;)gf(this,a,d),d+=this.blockSize;if("string"===typeof a)for(;d<b;){if(e[f]=a.charCodeAt(d),++f,++d,f==this.blockSize){gf(this,e);f=0;break}}else for(;d<b;)if(e[f]=a[d],++f,++d,f==this.blockSize){gf(this,e);f=0;break}}this.i=f;this.l+=b}};
ff.prototype.digest=function(){var a=[],b=8*this.l;56>this.i?this.update(this.j,56-this.i):this.update(this.j,this.blockSize-(this.i-56));for(var c=this.blockSize-1;56<=c;c--)this.o[c]=b&255,b/=256;gf(this,this.o);for(c=b=0;5>c;c++)for(var d=24;0<=d;d-=8)a[b]=this.h[c]>>d&255,++b;return a};function hf(a){return"string"==typeof a.className?a.className:a.getAttribute&&a.getAttribute("class")||""}
function jf(a,b){"string"==typeof a.className?a.className=b:a.setAttribute&&a.setAttribute("class",b)}
function kf(a,b){a.classList?b=a.classList.contains(b):(a=a.classList?a.classList:hf(a).match(/\S+/g)||[],b=0<=eb(a,b));return b}
function lf(){var a=document.body;a.classList?a.classList.remove("inverted-hdpi"):kf(a,"inverted-hdpi")&&jf(a,Array.prototype.filter.call(a.classList?a.classList:hf(a).match(/\S+/g)||[],function(b){return"inverted-hdpi"!=b}).join(" "))}
;var mf="StopIteration"in x?x.StopIteration:{message:"StopIteration",stack:""};function nf(){}
nf.prototype.da=function(){throw mf;};
nf.prototype.next=function(){return of};
var of={done:!0,value:void 0};function pf(a){return{value:a,done:!1}}
function qf(a){if(a.done)throw mf;return a.value}
nf.prototype.T=function(){return this};function rf(a){if(a instanceof sf||a instanceof tf||a instanceof uf)return a;if("function"==typeof a.da)return new sf(function(){return vf(a)});
if("function"==typeof a[Symbol.iterator])return new sf(function(){return a[Symbol.iterator]()});
if("function"==typeof a.T)return new sf(function(){return vf(a.T())});
throw Error("Not an iterator or iterable.");}
function vf(a){if(!(a instanceof nf))return a;var b=!1;return{next:function(){for(var c;!b;)try{c=a.da();break}catch(d){if(d!==mf)throw d;b=!0}return{value:c,done:b}}}}
function sf(a){this.h=a}
sf.prototype.T=function(){return new tf(this.h())};
sf.prototype[Symbol.iterator]=function(){return new uf(this.h())};
sf.prototype.i=function(){return new uf(this.h())};
function tf(a){this.h=a}
r(tf,nf);tf.prototype.da=function(){var a=this.h.next();if(a.done)throw mf;return a.value};
tf.prototype.next=function(){return this.h.next()};
tf.prototype[Symbol.iterator]=function(){return new uf(this.h)};
tf.prototype.i=function(){return new uf(this.h)};
function uf(a){sf.call(this,function(){return a});
this.j=a}
r(uf,sf);uf.prototype.next=function(){return this.j.next()};function wf(a,b){this.i={};this.h=[];this.ja=this.size=0;var c=arguments.length;if(1<c){if(c%2)throw Error("Uneven number of arguments");for(var d=0;d<c;d+=2)this.set(arguments[d],arguments[d+1])}else if(a)if(a instanceof wf)for(c=a.Za(),d=0;d<c.length;d++)this.set(c[d],a.get(c[d]));else for(d in a)this.set(d,a[d])}
m=wf.prototype;m.Za=function(){xf(this);return this.h.concat()};
m.has=function(a){return yf(this.i,a)};
m.equals=function(a,b){if(this===a)return!0;if(this.size!=a.size)return!1;b=b||zf;xf(this);for(var c,d=0;c=this.h[d];d++)if(!b(this.get(c),a.get(c)))return!1;return!0};
function zf(a,b){return a===b}
m.isEmpty=function(){return 0==this.size};
m.clear=function(){this.i={};this.ja=this.size=this.h.length=0};
m.remove=function(a){return this.delete(a)};
m.delete=function(a){return yf(this.i,a)?(delete this.i[a],--this.size,this.ja++,this.h.length>2*this.size&&xf(this),!0):!1};
function xf(a){if(a.size!=a.h.length){for(var b=0,c=0;b<a.h.length;){var d=a.h[b];yf(a.i,d)&&(a.h[c++]=d);b++}a.h.length=c}if(a.size!=a.h.length){var e={};for(c=b=0;b<a.h.length;)d=a.h[b],yf(e,d)||(a.h[c++]=d,e[d]=1),b++;a.h.length=c}}
m.get=function(a,b){return yf(this.i,a)?this.i[a]:b};
m.set=function(a,b){yf(this.i,a)||(this.size+=1,this.h.push(a),this.ja++);this.i[a]=b};
m.forEach=function(a,b){for(var c=this.Za(),d=0;d<c.length;d++){var e=c[d],f=this.get(e);a.call(b,f,e,this)}};
m.clone=function(){return new wf(this)};
m.keys=function(){return rf(this.T(!0)).i()};
m.values=function(){return rf(this.T(!1)).i()};
m.entries=function(){var a=this;return df(this.keys(),function(b){return[b,a.get(b)]})};
m.T=function(a){xf(this);var b=0,c=this.ja,d=this,e=new nf;e.next=function(){if(c!=d.ja)throw Error("The map has changed since the iterator was created");if(b>=d.h.length)return of;var g=d.h[b++];return pf(a?g:d.i[g])};
var f=e.next;e.da=function(){return qf(f.call(e))};
return e};
function yf(a,b){return Object.prototype.hasOwnProperty.call(a,b)}
;function Af(a){Bf();return Ib(a)}
var Bf=La;function Cf(a){var b=[];Df(new Ef,a,b);return b.join("")}
function Ef(){}
function Df(a,b,c){if(null==b)c.push("null");else{if("object"==typeof b){if(Array.isArray(b)){var d=b;b=d.length;c.push("[");for(var e="",f=0;f<b;f++)c.push(e),Df(a,d[f],c),e=",";c.push("]");return}if(b instanceof String||b instanceof Number||b instanceof Boolean)b=b.valueOf();else{c.push("{");e="";for(d in b)Object.prototype.hasOwnProperty.call(b,d)&&(f=b[d],"function"!=typeof f&&(c.push(e),Ff(d,c),c.push(":"),Df(a,f,c),e=","));c.push("}");return}}switch(typeof b){case "string":Ff(b,c);break;case "number":c.push(isFinite(b)&&
!isNaN(b)?String(b):"null");break;case "boolean":c.push(String(b));break;case "function":c.push("null");break;default:throw Error("Unknown type: "+typeof b);}}}
var Gf={'"':'\\"',"\\":"\\\\","/":"\\/","\b":"\\b","\f":"\\f","\n":"\\n","\r":"\\r","\t":"\\t","\v":"\\u000b"},Hf=/\uffff/.test("\uffff")?/[\\"\x00-\x1f\x7f-\uffff]/g:/[\\"\x00-\x1f\x7f-\xff]/g;function Ff(a,b){b.push('"',a.replace(Hf,function(c){var d=Gf[c];d||(d="\\u"+(c.charCodeAt(0)|65536).toString(16).substr(1),Gf[c]=d);return d}),'"')}
;function If(a){if(!a)return!1;try{return!!a.$goog_Thenable}catch(b){return!1}}
;function Jf(a){this.h=0;this.D=void 0;this.l=this.i=this.j=null;this.o=this.m=!1;if(a!=La)try{var b=this;a.call(void 0,function(c){Kf(b,2,c)},function(c){Kf(b,3,c)})}catch(c){Kf(this,3,c)}}
function Lf(){this.next=this.context=this.onRejected=this.i=this.h=null;this.j=!1}
Lf.prototype.reset=function(){this.context=this.onRejected=this.i=this.h=null;this.j=!1};
var Mf=new Qe(function(){return new Lf},function(a){a.reset()});
function Nf(a,b,c){var d=Mf.get();d.i=a;d.onRejected=b;d.context=c;return d}
function Of(a){return new Jf(function(b,c){c(a)})}
Jf.prototype.then=function(a,b,c){return Pf(this,"function"===typeof a?a:null,"function"===typeof b?b:null,c)};
Jf.prototype.$goog_Thenable=!0;function Qf(a,b){return Pf(a,null,b,void 0)}
Jf.prototype.cancel=function(a){if(0==this.h){var b=new Rf(a);Xe(function(){Sf(this,b)},this)}};
function Sf(a,b){if(0==a.h)if(a.j){var c=a.j;if(c.i){for(var d=0,e=null,f=null,g=c.i;g&&(g.j||(d++,g.h==a&&(e=g),!(e&&1<d)));g=g.next)e||(f=g);e&&(0==c.h&&1==d?Sf(c,b):(f?(d=f,d.next==c.l&&(c.l=d),d.next=d.next.next):Tf(c),Uf(c,e,3,b)))}a.j=null}else Kf(a,3,b)}
function Vf(a,b){a.i||2!=a.h&&3!=a.h||Wf(a);a.l?a.l.next=b:a.i=b;a.l=b}
function Pf(a,b,c,d){var e=Nf(null,null,null);e.h=new Jf(function(f,g){e.i=b?function(h){try{var k=b.call(d,h);f(k)}catch(l){g(l)}}:f;
e.onRejected=c?function(h){try{var k=c.call(d,h);void 0===k&&h instanceof Rf?g(h):f(k)}catch(l){g(l)}}:g});
e.h.j=a;Vf(a,e);return e.h}
Jf.prototype.A=function(a){this.h=0;Kf(this,2,a)};
Jf.prototype.L=function(a){this.h=0;Kf(this,3,a)};
function Kf(a,b,c){if(0==a.h){a===c&&(b=3,c=new TypeError("Promise cannot resolve to itself"));a.h=1;a:{var d=c,e=a.A,f=a.L;if(d instanceof Jf){Vf(d,Nf(e||La,f||null,a));var g=!0}else if(If(d))d.then(e,f,a),g=!0;else{if(Pa(d))try{var h=d.then;if("function"===typeof h){Xf(d,h,e,f,a);g=!0;break a}}catch(k){f.call(a,k);g=!0;break a}g=!1}}g||(a.D=c,a.h=b,a.j=null,Wf(a),3!=b||c instanceof Rf||Yf(a,c))}}
function Xf(a,b,c,d,e){function f(k){h||(h=!0,d.call(e,k))}
function g(k){h||(h=!0,c.call(e,k))}
var h=!1;try{b.call(a,g,f)}catch(k){f(k)}}
function Wf(a){a.m||(a.m=!0,Xe(a.u,a))}
function Tf(a){var b=null;a.i&&(b=a.i,a.i=b.next,b.next=null);a.i||(a.l=null);return b}
Jf.prototype.u=function(){for(var a;a=Tf(this);)Uf(this,a,this.h,this.D);this.m=!1};
function Uf(a,b,c,d){if(3==c&&b.onRejected&&!b.j)for(;a&&a.o;a=a.j)a.o=!1;if(b.h)b.h.j=null,Zf(b,c,d);else try{b.j?b.i.call(b.context):Zf(b,c,d)}catch(e){$f.call(null,e)}Re(Mf,b)}
function Zf(a,b,c){2==b?a.i.call(a.context,c):a.onRejected&&a.onRejected.call(a.context,c)}
function Yf(a,b){a.o=!0;Xe(function(){a.o&&$f.call(null,b)})}
var $f=md;function Rf(a){bb.call(this,a)}
Ya(Rf,bb);Rf.prototype.name="cancel";function J(a){I.call(this);this.u=1;this.l=[];this.m=0;this.i=[];this.j={};this.A=!!a}
Ya(J,I);m=J.prototype;m.subscribe=function(a,b,c){var d=this.j[a];d||(d=this.j[a]=[]);var e=this.u;this.i[e]=a;this.i[e+1]=b;this.i[e+2]=c;this.u=e+3;d.push(e);return e};
function ag(a,b,c,d){if(b=a.j[b]){var e=a.i;(b=b.find(function(f){return e[f+1]==c&&e[f+2]==d}))&&a.va(b)}}
m.va=function(a){var b=this.i[a];if(b){var c=this.j[b];0!=this.m?(this.l.push(a),this.i[a+1]=La):(c&&kb(c,a),delete this.i[a],delete this.i[a+1],delete this.i[a+2])}return!!b};
m.ka=function(a,b){var c=this.j[a];if(c){for(var d=Array(arguments.length-1),e=1,f=arguments.length;e<f;e++)d[e-1]=arguments[e];if(this.A)for(e=0;e<c.length;e++){var g=c[e];bg(this.i[g+1],this.i[g+2],d)}else{this.m++;try{for(e=0,f=c.length;e<f&&!this.h();e++)g=c[e],this.i[g+1].apply(this.i[g+2],d)}finally{if(this.m--,0<this.l.length&&0==this.m)for(;c=this.l.pop();)this.va(c)}}return 0!=e}return!1};
function bg(a,b,c){Xe(function(){a.apply(b,c)})}
m.clear=function(a){if(a){var b=this.j[a];b&&(b.forEach(this.va,this),delete this.j[a])}else this.i.length=0,this.j={}};
m.I=function(){J.aa.I.call(this);this.clear();this.l.length=0};function cg(a){this.h=a}
cg.prototype.set=function(a,b){void 0===b?this.h.remove(a):this.h.set(a,Cf(b))};
cg.prototype.get=function(a){try{var b=this.h.get(a)}catch(c){return}if(null!==b)try{return JSON.parse(b)}catch(c){throw"Storage: Invalid value was encountered";}};
cg.prototype.remove=function(a){this.h.remove(a)};function dg(a){this.h=a}
Ya(dg,cg);function eg(a){this.data=a}
function fg(a){return void 0===a||a instanceof eg?a:new eg(a)}
dg.prototype.set=function(a,b){dg.aa.set.call(this,a,fg(b))};
dg.prototype.i=function(a){a=dg.aa.get.call(this,a);if(void 0===a||a instanceof Object)return a;throw"Storage: Invalid value was encountered";};
dg.prototype.get=function(a){if(a=this.i(a)){if(a=a.data,void 0===a)throw"Storage: Invalid value was encountered";}else a=void 0;return a};function gg(a){this.h=a}
Ya(gg,dg);gg.prototype.set=function(a,b,c){if(b=fg(b)){if(c){if(c<Date.now()){gg.prototype.remove.call(this,a);return}b.expiration=c}b.creation=Date.now()}gg.aa.set.call(this,a,b)};
gg.prototype.i=function(a){var b=gg.aa.i.call(this,a);if(b){var c=b.creation,d=b.expiration;if(d&&d<Date.now()||c&&c>Date.now())gg.prototype.remove.call(this,a);else return b}};function hg(){}
;function ig(){}
Ya(ig,hg);ig.prototype[Symbol.iterator]=function(){return rf(this.T(!0)).i()};
ig.prototype.clear=function(){var a=Array.from(this);a=q(a);for(var b=a.next();!b.done;b=a.next())this.remove(b.value)};function jg(a){this.h=a}
Ya(jg,ig);m=jg.prototype;m.isAvailable=function(){if(!this.h)return!1;try{return this.h.setItem("__sak","1"),this.h.removeItem("__sak"),!0}catch(a){return!1}};
m.set=function(a,b){try{this.h.setItem(a,b)}catch(c){if(0==this.h.length)throw"Storage mechanism: Storage disabled";throw"Storage mechanism: Quota exceeded";}};
m.get=function(a){a=this.h.getItem(a);if("string"!==typeof a&&null!==a)throw"Storage mechanism: Invalid value was encountered";return a};
m.remove=function(a){this.h.removeItem(a)};
m.T=function(a){var b=0,c=this.h,d=new nf;d.next=function(){if(b>=c.length)return of;var f=c.key(b++);if(a)return pf(f);f=c.getItem(f);if("string"!==typeof f)throw"Storage mechanism: Invalid value was encountered";return pf(f)};
var e=d.next;d.da=function(){return qf(e.call(d))};
return d};
m.clear=function(){this.h.clear()};
m.key=function(a){return this.h.key(a)};function kg(){var a=null;try{a=window.localStorage||null}catch(b){}this.h=a}
Ya(kg,jg);function lg(a,b){this.i=a;this.h=null;var c;if(c=ic)c=!(9<=Number(vc));if(c){mg||(mg=new wf);this.h=mg.get(a);this.h||(b?this.h=document.getElementById(b):(this.h=document.createElement("userdata"),this.h.addBehavior("#default#userData"),document.body.appendChild(this.h)),mg.set(a,this.h));try{this.h.load(this.i)}catch(d){this.h=null}}}
Ya(lg,ig);var ng={".":".2E","!":".21","~":".7E","*":".2A","'":".27","(":".28",")":".29","%":"."},mg=null;function og(a){return"_"+encodeURIComponent(a).replace(/[.!~*'()%]/g,function(b){return ng[b]})}
m=lg.prototype;m.isAvailable=function(){return!!this.h};
m.set=function(a,b){this.h.setAttribute(og(a),b);pg(this)};
m.get=function(a){a=this.h.getAttribute(og(a));if("string"!==typeof a&&null!==a)throw"Storage mechanism: Invalid value was encountered";return a};
m.remove=function(a){this.h.removeAttribute(og(a));pg(this)};
m.T=function(a){var b=0,c=this.h.XMLDocument.documentElement.attributes,d=new nf;d.next=function(){if(b>=c.length)return of;var f=c[b++];if(a)return pf(decodeURIComponent(f.nodeName.replace(/\./g,"%")).substr(1));f=f.nodeValue;if("string"!==typeof f)throw"Storage mechanism: Invalid value was encountered";return pf(f)};
var e=d.next;d.da=function(){return qf(e.call(d))};
return d};
m.clear=function(){for(var a=this.h.XMLDocument.documentElement,b=a.attributes.length;0<b;b--)a.removeAttribute(a.attributes[b-1].nodeName);pg(this)};
function pg(a){try{a.h.save(a.i)}catch(b){throw"Storage mechanism: Quota exceeded";}}
;function qg(a,b){this.i=a;this.h=b+"::"}
Ya(qg,ig);qg.prototype.set=function(a,b){this.i.set(this.h+a,b)};
qg.prototype.get=function(a){return this.i.get(this.h+a)};
qg.prototype.remove=function(a){this.i.remove(this.h+a)};
qg.prototype.T=function(a){var b=this.i.T(!0),c=this,d=new nf;d.next=function(){try{var f=b.da()}catch(g){if(g===mf)return of;throw g;}for(;f.substr(0,c.h.length)!=c.h;)try{f=b.da()}catch(g){if(g===mf)return of;throw g;}return pf(a?f.substr(c.h.length):c.i.get(f))};
var e=d.next;d.da=function(){return qf(e.call(d))};
return d};function rg(a){H.call(this,a)}
r(rg,H);rg.prototype.getKey=function(){return Pc(this,1)};
rg.prototype.getValue=function(){return Pc(this,2===Sc(this,sg)?2:-1)};
rg.prototype.setValue=function(a){return Rc(this,2,sg,a)};
var sg=[2,3,4,5,6];function tg(a){H.call(this,a)}
r(tg,H);function ug(a){H.call(this,a)}
r(ug,H);function vg(a){H.call(this,a)}
r(vg,H);function wg(a){H.call(this,a,-1,xg)}
r(wg,H);wg.prototype.getPlayerType=function(){return Pc(this,36)};
wg.prototype.setHomeGroupInfo=function(a){return G(this,81,a)};
var xg=[9,66,24,32,86,100,101];function yg(a){H.call(this,a,-1,zg)}
r(yg,H);var zg=[15,26,28];function Ag(a){H.call(this,a)}
r(Ag,H);Ag.prototype.setToken=function(a){return F(this,2,a)};function Bg(a){H.call(this,a,-1,Cg)}
r(Bg,H);Bg.prototype.setSafetyMode=function(a){return F(this,5,a)};
var Cg=[12];function Dg(a){H.call(this,a,-1,Eg)}
r(Dg,H);var Eg=[12];function Fg(a){H.call(this,a)}
r(Fg,H);var Gg={vh:0,gh:1,mh:2,nh:4,sh:8,oh:16,ph:32,uh:64,th:128,ih:256,kh:512,rh:1024,jh:2048,lh:4096,hh:8192,qh:16384};function Hg(a){H.call(this,a)}
r(Hg,H);function Ig(a,b){G(a,1,b)}
Hg.prototype.Y=function(a){F(this,2,a)};
function Jg(a){H.call(this,a)}
r(Jg,H);function Kg(a,b){G(a,1,b)}
;function Lg(a){H.call(this,a,-1,Mg)}
r(Lg,H);Lg.prototype.Y=function(a){F(this,1,a)};
function Ng(a,b){G(a,2,b)}
var Mg=[3];function Og(a){H.call(this,a)}
r(Og,H);Og.prototype.Y=function(a){F(this,1,a)};function Pg(a){H.call(this,a)}
r(Pg,H);Pg.prototype.Y=function(a){F(this,1,a)};function Qg(a){H.call(this,a)}
r(Qg,H);Qg.prototype.Y=function(a){F(this,1,a)};function Rg(a){H.call(this,a)}
r(Rg,H);function Sg(a){H.call(this,a)}
r(Sg,H);function Tg(a){H.call(this,a,-1,Ug)}
r(Tg,H);Tg.prototype.getPlayerType=function(){var a=Pc(this,7);return null==a?0:a};
Tg.prototype.setVideoId=function(a){return F(this,19,a)};
function Vg(a,b){Xc(a,68,Wg,b)}
function Wg(a){H.call(this,a)}
r(Wg,H);Wg.prototype.getId=function(){var a=Pc(this,2);return null==a?"":a};
var Ug=[83,68];function Xg(a){H.call(this,a)}
r(Xg,H);function Yg(a){H.call(this,a)}
r(Yg,H);function Zg(a){H.call(this,a)}
r(Zg,H);function $g(a){H.call(this,a,428)}
r($g,H);
var Wc=[23,24,11,6,7,5,2,3,20,21,28,32,37,229,241,45,59,225,288,72,73,78,208,156,202,215,74,76,79,80,111,85,91,97,100,102,105,119,126,127,136,146,157,158,159,163,164,168,176,222,383,177,178,179,411,184,188,189,190,191,193,194,195,196,198,199,200,201,203,204,205,206,258,259,260,261,209,226,227,232,233,234,240,247,248,251,254,255,270,278,291,293,300,304,308,309,310,311,313,314,319,321,323,324,328,330,331,332,337,338,340,344,348,350,351,352,353,354,355,356,357,358,361,363,364,368,369,370,373,374,375,
378,380,381,388,389,403,412,413,414,415,416,417,418,419,420,423,424,425,426,427,117];function ah(a){H.call(this,a)}
r(ah,H);function bh(a){H.call(this,a)}
r(bh,H);bh.prototype.setVideoId=function(a){return Rc(this,1,ch,a)};
bh.prototype.getPlaylistId=function(){return Pc(this,2===Sc(this,ch)?2:-1)};
var ch=[1,2];function dh(a){H.call(this,a,-1,eh)}
r(dh,H);var eh=[3];function fh(a){H.call(this,a,1)}
r(fh,H);function gh(a){H.call(this,a)}
r(gh,H);var hh;hh=new function(a,b){this.h=a;this.fieldName=b;this.isRepeated=0;this.i=ld}(406606992,{zo:0},gh);function ih(){gh.apply(this,arguments)}
r(ih,gh);ae(ih);function jh(a,b){1<b.length?a[b[0]]=b[1]:1===b.length&&Object.assign(a,b[0])}
;var kh,lh,mh,nh=x.window,oh=(null===(kh=null===nh||void 0===nh?void 0:nh.yt)||void 0===kh?void 0:kh.config_)||(null===(lh=null===nh||void 0===nh?void 0:nh.ytcfg)||void 0===lh?void 0:lh.data_)||{},ph=(null===(mh=null===nh||void 0===nh?void 0:nh.ytcfg)||void 0===mh?void 0:mh.obfuscatedData_)||[];function qh(){fh.apply(this,arguments)}
r(qh,fh);ae(qh);var rh=new qh(ph),sh=oh.EXPERIMENT_FLAGS;if(!sh||!sh.jspb_i18n_extension){var th=new ih;hh.i(rh,th)}z("yt.config_",oh,void 0);z("yt.configJspb_",ph,void 0);function uh(){jh(oh,arguments)}
function B(a,b){return a in oh?oh[a]:b}
function vh(a){return B(a,void 0)}
;function K(a){a=wh(a);return"string"===typeof a&&"false"===a?!1:!!a}
function xh(a,b){a=wh(a);return void 0===a&&void 0!==b?b:Number(a||0)}
function yh(){return B("EXPERIMENTS_TOKEN","")}
function wh(a){var b=B("EXPERIMENTS_FORCED_FLAGS",{});return void 0!==b[a]?b[a]:B("EXPERIMENT_FLAGS",{})[a]}
function zh(){var a=[],b=B("EXPERIMENTS_FORCED_FLAGS",{});for(c in b)a.push({key:c,value:String(b[c])});var c=B("EXPERIMENT_FLAGS",{});for(var d in c)d.startsWith("force_")&&void 0===b[d]&&a.push({key:d,value:String(c[d])});return a}
;var Ah={appSettingsCaptured:!0,visualElementAttached:!0,visualElementGestured:!0,visualElementHidden:!0,visualElementShown:!0,flowEvent:!0,visualElementStateChanged:!0,playbackAssociated:!0,youThere:!0,accountStateChangeSignedIn:!0,accountStateChangeSignedOut:!0},Bh={latencyActionBaselined:!0,latencyActionInfo:!0,latencyActionTicked:!0,bedrockRepetitiveActionTimed:!0,adsClientStateChange:!0,streamzIncremented:!0,mdxDialAdditionalDataUpdateEvent:!0,tvhtml5WatchKeyEvent:!0,tvhtml5VideoSeek:!0,tokenRefreshEvent:!0,
adNotify:!0,adNotifyFilled:!0,tvhtml5LaunchUrlComponentChanged:!0,bedrockResourceConsumptionSnapshot:!0,deviceStartupMetrics:!0,mdxSignIn:!0,tvhtml5KeyboardLogging:!0,tvhtml5StartupSoundEvent:!0,tvhtml5LiveChatStatus:!0,tvhtml5DeviceStorageStatus:!0,tvhtml5LocalStorage:!0,directSignInEvent:!0,finalPayload:!0,tvhtml5SearchCompleted:!0,tvhtml5KeyboardPerformance:!0,adNotifyFailure:!0,latencyActionSpan:!0,tvhtml5AccountDialogOpened:!0,tvhtml5ApiTest:!0};var Ch=0,Dh=lc?"webkit":kc?"moz":ic?"ms":hc?"o":"";z("ytDomDomGetNextId",A("ytDomDomGetNextId")||function(){return++Ch},void 0);var Eh=[];function Fh(a){Eh.forEach(function(b){return b(a)})}
function Gh(a){return a&&window.yterr?function(){try{return a.apply(this,arguments)}catch(b){M(b)}}:a}
function M(a,b,c,d){var e=A("yt.logging.errors.log");e?e(a,"ERROR",b,c,d):(e=B("ERRORS",[]),e.push([a,"ERROR",b,c,d]),uh("ERRORS",e));Fh(a)}
function Hh(a,b,c,d){var e=A("yt.logging.errors.log");e?e(a,"WARNING",b,c,d):(e=B("ERRORS",[]),e.push([a,"WARNING",b,c,d]),uh("ERRORS",e))}
;var Ih={stopImmediatePropagation:1,stopPropagation:1,preventMouseEvent:1,preventManipulation:1,preventDefault:1,layerX:1,layerY:1,screenX:1,screenY:1,scale:1,rotation:1,webkitMovementX:1,webkitMovementY:1};
function Jh(a){this.type="";this.state=this.source=this.data=this.currentTarget=this.relatedTarget=this.target=null;this.charCode=this.keyCode=0;this.metaKey=this.shiftKey=this.ctrlKey=this.altKey=!1;this.rotation=this.clientY=this.clientX=0;this.scale=1;this.changedTouches=this.touches=null;try{if(a=a||window.event){this.event=a;for(var b in a)b in Ih||(this[b]=a[b]);this.scale=a.scale;this.rotation=a.rotation;var c=a.target||a.srcElement;c&&3==c.nodeType&&(c=c.parentNode);this.target=c;var d=a.relatedTarget;
if(d)try{d=d.nodeName?d:null}catch(e){d=null}else"mouseover"==this.type?d=a.fromElement:"mouseout"==this.type&&(d=a.toElement);this.relatedTarget=d;this.clientX=void 0!=a.clientX?a.clientX:a.pageX;this.clientY=void 0!=a.clientY?a.clientY:a.pageY;this.keyCode=a.keyCode?a.keyCode:a.which;this.charCode=a.charCode||("keypress"==this.type?this.keyCode:0);this.altKey=a.altKey;this.ctrlKey=a.ctrlKey;this.shiftKey=a.shiftKey;this.metaKey=a.metaKey;this.h=a.pageX;this.i=a.pageY}}catch(e){}}
function Kh(a){if(document.body&&document.documentElement){var b=document.body.scrollTop+document.documentElement.scrollTop;a.h=a.clientX+(document.body.scrollLeft+document.documentElement.scrollLeft);a.i=a.clientY+b}}
Jh.prototype.preventDefault=function(){this.event&&(this.event.returnValue=!1,this.event.preventDefault&&this.event.preventDefault())};
Jh.prototype.stopPropagation=function(){this.event&&(this.event.cancelBubble=!0,this.event.stopPropagation&&this.event.stopPropagation())};
Jh.prototype.stopImmediatePropagation=function(){this.event&&(this.event.cancelBubble=!0,this.event.stopImmediatePropagation&&this.event.stopImmediatePropagation())};var pb=x.ytEventsEventsListeners||{};z("ytEventsEventsListeners",pb,void 0);var Lh=x.ytEventsEventsCounter||{count:0};z("ytEventsEventsCounter",Lh,void 0);
function Mh(a,b,c,d){d=void 0===d?{}:d;a.addEventListener&&("mouseenter"!=b||"onmouseenter"in document?"mouseleave"!=b||"onmouseenter"in document?"mousewheel"==b&&"MozBoxSizing"in document.documentElement.style&&(b="MozMousePixelScroll"):b="mouseout":b="mouseover");return ob(function(e){var f="boolean"===typeof e[4]&&e[4]==!!d,g=Pa(e[4])&&Pa(d)&&tb(e[4],d);return!!e.length&&e[0]==a&&e[1]==b&&e[2]==c&&(f||g)})}
var Nh=db(function(){var a=!1;try{var b=Object.defineProperty({},"capture",{get:function(){a=!0}});
window.addEventListener("test",null,b)}catch(c){}return a});
function Oh(a,b,c,d){d=void 0===d?{}:d;if(!a||!a.addEventListener&&!a.attachEvent)return"";var e=Mh(a,b,c,d);if(e)return e;e=++Lh.count+"";var f=!("mouseenter"!=b&&"mouseleave"!=b||!a.addEventListener||"onmouseenter"in document);var g=f?function(h){h=new Jh(h);if(!Bd(h.relatedTarget,function(k){return k==a}))return h.currentTarget=a,h.type=b,c.call(a,h)}:function(h){h=new Jh(h);
h.currentTarget=a;return c.call(a,h)};
g=Gh(g);a.addEventListener?("mouseenter"==b&&f?b="mouseover":"mouseleave"==b&&f?b="mouseout":"mousewheel"==b&&"MozBoxSizing"in document.documentElement.style&&(b="MozMousePixelScroll"),Nh()||"boolean"===typeof d?a.addEventListener(b,g,d):a.addEventListener(b,g,!!d.capture)):a.attachEvent("on"+b,g);pb[e]=[a,b,c,g,d];return e}
function Ph(a){a&&("string"==typeof a&&(a=[a]),fb(a,function(b){if(b in pb){var c=pb[b],d=c[0],e=c[1],f=c[3];c=c[4];d.removeEventListener?Nh()||"boolean"===typeof c?d.removeEventListener(e,f,c):d.removeEventListener(e,f,!!c.capture):d.detachEvent&&d.detachEvent("on"+e,f);delete pb[b]}}))}
;var Qh=window.ytcsi&&window.ytcsi.now?window.ytcsi.now:window.performance&&window.performance.timing&&window.performance.now&&window.performance.timing.navigationStart?function(){return window.performance.timing.navigationStart+window.performance.now()}:function(){return(new Date).getTime()};function Rh(a,b){"function"===typeof a&&(a=Gh(a));return window.setTimeout(a,b)}
function Sh(a){window.clearTimeout(a)}
;function Th(a){this.L=a;this.i=null;this.m=0;this.A=null;this.u=0;this.j=[];for(a=0;4>a;a++)this.j.push(0);this.l=0;this.S=Oh(window,"mousemove",Va(this.Z,this));a=Va(this.M,this);"function"===typeof a&&(a=Gh(a));this.ga=window.setInterval(a,25)}
Ya(Th,I);Th.prototype.Z=function(a){void 0===a.h&&Kh(a);var b=a.h;void 0===a.i&&Kh(a);this.i=new xd(b,a.i)};
Th.prototype.M=function(){if(this.i){var a=Qh();if(0!=this.m){var b=this.A,c=this.i,d=b.x-c.x;b=b.y-c.y;d=Math.sqrt(d*d+b*b)/(a-this.m);this.j[this.l]=.5<Math.abs((d-this.u)/this.u)?1:0;for(c=b=0;4>c;c++)b+=this.j[c]||0;3<=b&&this.L();this.u=d}this.m=a;this.A=this.i;this.l=(this.l+1)%4}};
Th.prototype.I=function(){window.clearInterval(this.ga);Ph(this.S)};function Uh(){}
function Vh(a,b){return Wh(a,0,b)}
Uh.prototype.N=function(a,b){return Wh(a,1,b)};
function Xh(a,b){Wh(a,2,b)}
function Yh(a){var b=A("yt.scheduler.instance.addImmediateJob");b?b(a):a()}
;function Zh(){Uh.apply(this,arguments)}
r(Zh,Uh);function $h(){Zh.h||(Zh.h=new Zh);return Zh.h}
function Wh(a,b,c){void 0!==c&&Number.isNaN(Number(c))&&(c=void 0);var d=A("yt.scheduler.instance.addJob");return d?d(a,b,c):void 0===c?(a(),NaN):Rh(a,c||0)}
Zh.prototype.U=function(a){if(void 0===a||!Number.isNaN(Number(a))){var b=A("yt.scheduler.instance.cancelJob");b?b(a):Sh(a)}};
Zh.prototype.start=function(){var a=A("yt.scheduler.instance.start");a&&a()};
Zh.prototype.pause=function(){var a=A("yt.scheduler.instance.pause");a&&a()};var ai=$h();var bi={};
function ci(a){var b=void 0===a?{}:a;a=void 0===b.ec?!1:b.ec;b=void 0===b.Tb?!0:b.Tb;if(null==A("_lact",window)){var c=parseInt(B("LACT"),10);c=isFinite(c)?Date.now()-Math.max(c,0):-1;z("_lact",c,window);z("_fact",c,window);-1==c&&di();Oh(document,"keydown",di);Oh(document,"keyup",di);Oh(document,"mousedown",di);Oh(document,"mouseup",di);a?Oh(window,"touchmove",function(){ei("touchmove",200)},{passive:!0}):(Oh(window,"resize",function(){ei("resize",200)}),b&&Oh(window,"scroll",function(){ei("scroll",200)}));
new Th(function(){ei("mouse",100)});
Oh(document,"touchstart",di,{passive:!0});Oh(document,"touchend",di,{passive:!0})}}
function ei(a,b){bi[a]||(bi[a]=!0,ai.N(function(){di();bi[a]=!1},b))}
function di(){null==A("_lact",window)&&ci();var a=Date.now();z("_lact",a,window);-1==A("_fact",window)&&z("_fact",a,window);(a=A("ytglobal.ytUtilActivityCallback_"))&&a()}
function fi(){var a=A("_lact",window);return null==a?-1:Math.max(Date.now()-a,0)}
;function gi(){var a=hi;A("yt.ads.biscotti.getId_")||z("yt.ads.biscotti.getId_",a,void 0)}
function ii(a){z("yt.ads.biscotti.lastId_",a,void 0)}
;var ji=/^[\w.]*$/,ki={q:!0,search_query:!0};function li(a,b){b=a.split(b);for(var c={},d=0,e=b.length;d<e;d++){var f=b[d].split("=");if(1==f.length&&f[0]||2==f.length)try{var g=mi(f[0]||""),h=mi(f[1]||"");g in c?Array.isArray(c[g])?mb(c[g],h):c[g]=[c[g],h]:c[g]=h}catch(t){var k=t,l=f[0],n=String(li);k.args=[{key:l,value:f[1],query:a,method:ni==n?"unchanged":n}];ki.hasOwnProperty(l)||Hh(k)}}return c}
var ni=String(li);function oi(a){var b=[];nb(a,function(c,d){var e=encodeURIComponent(String(d)),f;Array.isArray(c)?f=c:f=[c];fb(f,function(g){""==g?b.push(e):b.push(e+"="+encodeURIComponent(String(g)))})});
return b.join("&")}
function pi(a){"?"==a.charAt(0)&&(a=a.substr(1));return li(a,"&")}
function qi(a){return-1!=a.indexOf("?")?(a=(a||"").split("#")[0],a=a.split("?",2),pi(1<a.length?a[1]:a[0])):{}}
function ri(a,b,c){var d=a.split("#",2);a=d[0];d=1<d.length?"#"+d[1]:"";var e=a.split("?",2);a=e[0];e=pi(e[1]||"");for(var f in b)!c&&null!==e&&f in e||(e[f]=b[f]);return dc(a,e)+d}
function si(a){if(!b)var b=window.location.href;var c=a.match(Yb)[1]||null,d=$b(a);c&&d?(a=a.match(Yb),b=b.match(Yb),a=a[3]==b[3]&&a[1]==b[1]&&a[4]==b[4]):a=d?$b(b)==d&&(Number(b.match(Yb)[4]||null)||null)==(Number(a.match(Yb)[4]||null)||null):!0;return a}
function mi(a){return a&&a.match(ji)?a:decodeURIComponent(a.replace(/\+/g," "))}
;function ti(a){var b=ui;a=void 0===a?A("yt.ads.biscotti.lastId_")||"":a;var c=Object,d=c.assign,e={};e.dt=Nd;e.flash="0";a:{try{var f=b.h.top.location.href}catch(lb){f=2;break a}f=f?f===b.i.location.href?0:1:2}e=(e.frm=f,e);try{e.u_tz=-(new Date).getTimezoneOffset();var g=void 0===g?qd:g;try{var h=g.history.length}catch(lb){h=0}e.u_his=h;var k;e.u_h=null==(k=qd.screen)?void 0:k.height;var l;e.u_w=null==(l=qd.screen)?void 0:l.width;var n;e.u_ah=null==(n=qd.screen)?void 0:n.availHeight;var t;e.u_aw=
null==(t=qd.screen)?void 0:t.availWidth;var y;e.u_cd=null==(y=qd.screen)?void 0:y.colorDepth}catch(lb){}h=b.h;try{var u=h.screenX;var D=h.screenY}catch(lb){}try{var E=h.outerWidth;var L=h.outerHeight}catch(lb){}try{var P=h.innerWidth;var R=h.innerHeight}catch(lb){}try{var V=h.screenLeft;var aa=h.screenTop}catch(lb){}try{P=h.innerWidth,R=h.innerHeight}catch(lb){}try{var Cd=h.screen.availWidth;var ea=h.screen.availTop}catch(lb){}u=[V,aa,u,D,Cd,ea,E,L,P,R];try{var ja=(b.h.top||window).document,Ga="CSS1Compat"==
ja.compatMode?ja.documentElement:ja.body;var $a=(new yd(Ga.clientWidth,Ga.clientHeight)).round()}catch(lb){$a=new yd(-12245933,-12245933)}ja=$a;$a={};var ab=void 0===ab?x:ab;Ga=new Oe;ab.SVGElement&&ab.document.createElementNS&&Ga.set(0);D=Hd();D["allow-top-navigation-by-user-activation"]&&Ga.set(1);D["allow-popups-to-escape-sandbox"]&&Ga.set(2);ab.crypto&&ab.crypto.subtle&&Ga.set(3);ab.TextDecoder&&ab.TextEncoder&&Ga.set(4);ab=Pe(Ga);$a.bc=ab;$a.bih=ja.height;$a.biw=ja.width;$a.brdim=u.join();b=
b.i;b=($a.vis=b.prerendering?3:{visible:1,hidden:2,prerender:3,preview:4,unloaded:5}[b.visibilityState||b.webkitVisibilityState||b.mozVisibilityState||""]||0,$a.wgl=!!qd.WebGLRenderingContext,$a);c=d.call(c,e,b);c.ca_type="image";a&&(c.bid=a);return c}
var ui=new function(){var a=window.document;this.h=window;this.i=a};
z("yt.ads_.signals_.getAdSignalsString",function(a){return oi(ti(a))},void 0);Date.now();var vi="XMLHttpRequest"in x?function(){return new XMLHttpRequest}:null;
function wi(){if(!vi)return null;var a=vi();return"open"in a?a:null}
function xi(a){switch(a&&"status"in a?a.status:-1){case 200:case 201:case 202:case 203:case 204:case 205:case 206:case 304:return!0;default:return!1}}
;var yi={Authorization:"AUTHORIZATION","X-Goog-Visitor-Id":"SANDBOXED_VISITOR_ID","X-Youtube-Domain-Admin-State":"DOMAIN_ADMIN_STATE","X-Youtube-Chrome-Connected":"CHROME_CONNECTED_HEADER","X-YouTube-Client-Name":"INNERTUBE_CONTEXT_CLIENT_NAME","X-YouTube-Client-Version":"INNERTUBE_CONTEXT_CLIENT_VERSION","X-YouTube-Delegation-Context":"INNERTUBE_CONTEXT_SERIALIZED_DELEGATION_CONTEXT","X-YouTube-Device":"DEVICE","X-Youtube-Identity-Token":"ID_TOKEN","X-YouTube-Page-CL":"PAGE_CL","X-YouTube-Page-Label":"PAGE_BUILD_LABEL",
"X-YouTube-Variants-Checksum":"VARIANTS_CHECKSUM"},zi="app debugcss debugjs expflag force_ad_params force_ad_encrypted force_viral_ad_response_params forced_experiments innertube_snapshots innertube_goldens internalcountrycode internalipoverride absolute_experiments conditional_experiments sbb sr_bns_address".split(" ").concat(ia("client_dev_mss_url client_dev_regex_map client_dev_root_url client_rollout_override expflag jsfeat jsmode mods".split(" "))),Ai=!1;
function Bi(a,b){b=void 0===b?{}:b;var c=si(a),d=K("web_ajax_ignore_global_headers_if_set"),e;for(e in yi){var f=B(yi[e]);!f||!c&&$b(a)||d&&void 0!==b[e]||(b[e]=f)}if(c||!$b(a))b["X-YouTube-Utc-Offset"]=String(-(new Date).getTimezoneOffset());if(c||!$b(a)){try{var g=(new Intl.DateTimeFormat).resolvedOptions().timeZone}catch(h){}g&&(b["X-YouTube-Time-Zone"]=g)}if(c||!$b(a))b["X-YouTube-Ad-Signals"]=oi(ti(void 0));return b}
function Ci(a){var b=window.location.search,c=$b(a);K("debug_handle_relative_url_for_query_forward_killswitch")||c||!si(a)||(c=document.location.hostname);var d=Zb(a.match(Yb)[5]||null);d=(c=c&&(c.endsWith("youtube.com")||c.endsWith("youtube-nocookie.com")))&&d&&d.startsWith("/api/");if(!c||d)return a;var e=pi(b),f={};fb(zi,function(g){e[g]&&(f[g]=e[g])});
return ri(a,f||{},!1)}
function Di(a,b){var c=b.format||"JSON";a=Ei(a,b);var d=Fi(a,b),e=!1,f=Gi(a,function(k){if(!e){e=!0;h&&Sh(h);var l=xi(k),n=null,t=400<=k.status&&500>k.status,y=500<=k.status&&600>k.status;if(l||t||y)n=Hi(a,c,k,b.convertToSafeHtml);if(l)a:if(k&&204==k.status)l=!0;else{switch(c){case "XML":l=0==parseInt(n&&n.return_code,10);break a;case "RAW":l=!0;break a}l=!!n}n=n||{};t=b.context||x;l?b.onSuccess&&b.onSuccess.call(t,k,n):b.onError&&b.onError.call(t,k,n);b.onFinish&&b.onFinish.call(t,k,n)}},b.method,
d,b.headers,b.responseType,b.withCredentials);
if(b.onTimeout&&0<b.timeout){var g=b.onTimeout;var h=Rh(function(){e||(e=!0,f.abort(),Sh(h),g.call(b.context||x,f))},b.timeout)}return f}
function Ei(a,b){b.includeDomain&&(a=document.location.protocol+"//"+document.location.hostname+(document.location.port?":"+document.location.port:"")+a);var c=B("XSRF_FIELD_NAME",void 0);if(b=b.urlParams)b[c]&&delete b[c],a=ri(a,b||{},!0);return a}
function Fi(a,b){var c=B("XSRF_FIELD_NAME",void 0),d=B("XSRF_TOKEN",void 0),e=b.postBody||"",f=b.postParams,g=vh("XSRF_FIELD_NAME"),h;b.headers&&(h=b.headers["Content-Type"]);b.excludeXsrf||$b(a)&&!b.withCredentials&&$b(a)!=document.location.hostname||"POST"!=b.method||h&&"application/x-www-form-urlencoded"!=h||b.postParams&&b.postParams[g]||(f||(f={}),f[c]=d);f&&"string"===typeof e&&(e=pi(e),xb(e,f),e=b.postBodyFormat&&"JSON"==b.postBodyFormat?JSON.stringify(e):cc(e));f=e||f&&!qb(f);!Ai&&f&&"POST"!=
b.method&&(Ai=!0,M(Error("AJAX request with postData should use POST")));return e}
function Hi(a,b,c,d){var e=null;switch(b){case "JSON":try{var f=c.responseText}catch(g){throw d=Error("Error reading responseText"),d.params=a,Hh(d),g;}a=c.getResponseHeader("Content-Type")||"";f&&0<=a.indexOf("json")&&(")]}'\n"===f.substring(0,5)&&(f=f.substring(5)),e=JSON.parse(f));break;case "XML":if(a=(a=c.responseXML)?Ii(a):null)e={},fb(a.getElementsByTagName("*"),function(g){e[g.tagName]=Ji(g)})}d&&Ki(e);
return e}
function Ki(a){if(Pa(a))for(var b in a){var c;(c="html_content"==b)||(c=b.length-5,c=0<=c&&b.indexOf("_html",c)==c);if(c){c=b;Cb("HTML that is escaped and sanitized server-side and passed through yt.net.ajax");var d=a[b],e=zb();d=e?e.createHTML(d):d;a[c]=new Rb(d)}else Ki(a[b])}}
function Ii(a){return a?(a=("responseXML"in a?a.responseXML:a).getElementsByTagName("root"))&&0<a.length?a[0]:null:null}
function Ji(a){var b="";fb(a.childNodes,function(c){b+=c.nodeValue});
return b}
function Gi(a,b,c,d,e,f,g){function h(){4==(k&&"readyState"in k?k.readyState:0)&&b&&Gh(b)(k)}
c=void 0===c?"GET":c;d=void 0===d?"":d;var k=wi();if(!k)return null;"onloadend"in k?k.addEventListener("loadend",h,!1):k.onreadystatechange=h;K("debug_forward_web_query_parameters")&&(a=Ci(a));k.open(c,a,!0);f&&(k.responseType=f);g&&(k.withCredentials=!0);c="POST"==c&&(void 0===window.FormData||!(d instanceof FormData));if(e=Bi(a,e))for(var l in e)k.setRequestHeader(l,e[l]),"content-type"==l.toLowerCase()&&(c=!1);c&&k.setRequestHeader("Content-Type","application/x-www-form-urlencoded");k.send(d);
return k}
;var Li=x.ytPubsubPubsubInstance||new J,Mi=x.ytPubsubPubsubSubscribedKeys||{},Ni=x.ytPubsubPubsubTopicToKeys||{},Oi=x.ytPubsubPubsubIsSynchronous||{};function Pi(a,b){var c=Qi();if(c&&b){var d=c.subscribe(a,function(){var e=arguments;var f=function(){Mi[d]&&b.apply&&"function"==typeof b.apply&&b.apply(window,e)};
try{Oi[a]?f():Rh(f,0)}catch(g){M(g)}},void 0);
Mi[d]=!0;Ni[a]||(Ni[a]=[]);Ni[a].push(d);return d}return 0}
function Ri(a){var b=Qi();b&&("number"===typeof a?a=[a]:"string"===typeof a&&(a=[parseInt(a,10)]),fb(a,function(c){b.unsubscribeByKey(c);delete Mi[c]}))}
function Si(a,b){var c=Qi();c&&c.publish.apply(c,arguments)}
function Ti(a){var b=Qi();if(b)if(b.clear(a),a)Ui(a);else for(var c in Ni)Ui(c)}
function Qi(){return x.ytPubsubPubsubInstance}
function Ui(a){Ni[a]&&(a=Ni[a],fb(a,function(b){Mi[b]&&delete Mi[b]}),a.length=0)}
J.prototype.subscribe=J.prototype.subscribe;J.prototype.unsubscribeByKey=J.prototype.va;J.prototype.publish=J.prototype.ka;J.prototype.clear=J.prototype.clear;z("ytPubsubPubsubInstance",Li,void 0);z("ytPubsubPubsubTopicToKeys",Ni,void 0);z("ytPubsubPubsubIsSynchronous",Oi,void 0);z("ytPubsubPubsubSubscribedKeys",Mi,void 0);function N(a){var b=Fa.apply(1,arguments);var c=Error.call(this,a);this.message=c.message;"stack"in c&&(this.stack=c.stack);this.args=[].concat(ia(b))}
r(N,Error);var Vi=window,O=Vi.ytcsi&&Vi.ytcsi.now?Vi.ytcsi.now:Vi.performance&&Vi.performance.timing&&Vi.performance.now&&Vi.performance.timing.navigationStart?function(){return Vi.performance.timing.navigationStart+Vi.performance.now()}:function(){return(new Date).getTime()};var Wi=xh("initial_gel_batch_timeout",2E3),Xi=Math.pow(2,16)-1,Yi=void 0;function Zi(){this.j=this.h=this.i=0}
var $i=new Zi,aj=new Zi,bj=!0,cj=x.ytLoggingTransportGELQueue_||new Map;z("ytLoggingTransportGELQueue_",cj,void 0);var dj=x.ytLoggingTransportGELProtoQueue_||new Map;z("ytLoggingTransportGELProtoQueue_",dj,void 0);var ej=x.ytLoggingTransportTokensToCttTargetIds_||{};z("ytLoggingTransportTokensToCttTargetIds_",ej,void 0);var fj=x.ytLoggingTransportTokensToJspbCttTargetIds_||{};z("ytLoggingTransportTokensToJspbCttTargetIds_",fj,void 0);
function gj(a,b){if("log_event"===a.endpoint){var c=hj(a),d=cj.get(c)||[];cj.set(c,d);d.push(a.payload);ij(b,d,c)}}
function jj(a,b){if("log_event"===a.endpoint){var c=hj(a,!0),d=dj.get(c)||[];dj.set(c,d);a=gd(a.payload);d.push(a);ij(b,d,c,!0)}}
function ij(a,b,c,d){d=void 0===d?!1:d;a&&(Yi=new a);a=xh("tvhtml5_logging_max_batch")||xh("web_logging_max_batch")||100;var e=O(),f=d?aj.j:$i.j;b.length>=a?kj({writeThenSend:!0},K("flush_only_full_queue")?c:void 0,d):10<=e-f&&(lj(d),d?aj.j=e:$i.j=e)}
function mj(a,b){if("log_event"===a.endpoint){var c=hj(a),d=new Map;d.set(c,[a.payload]);b&&(Yi=new b);return new Jf(function(e){Yi&&Yi.isReady()?nj(d,e,{bypassNetworkless:!0},!0):e()})}}
function oj(a,b){if("log_event"===a.endpoint){var c=hj(a,!0),d=new Map;d.set(c,[gd(a.payload)]);b&&(Yi=new b);return new Jf(function(e){Yi&&Yi.isReady()?pj(d,e,{bypassNetworkless:!0},!0):e()})}}
function hj(a,b){var c="";if(a.wa)c="visitorOnlyApprovedKey";else if(a.cttAuthInfo){if(void 0===b?0:b){b=a.cttAuthInfo.token;c=a.cttAuthInfo;var d=new bh;c.videoId?d.setVideoId(c.videoId):c.playlistId&&Rc(d,2,ch,c.playlistId);fj[b]=d}else b=a.cttAuthInfo,c={},b.videoId?c.videoId=b.videoId:b.playlistId&&(c.playlistId=b.playlistId),ej[a.cttAuthInfo.token]=c;c=a.cttAuthInfo.token}return c}
function kj(a,b,c){a=void 0===a?{}:a;c=void 0===c?!1:c;new Jf(function(d){c?(Sh(aj.i),Sh(aj.h),aj.h=0):(Sh($i.i),Sh($i.h),$i.h=0);if(Yi&&Yi.isReady())if(void 0!==b)if(c){var e=new Map,f=dj.get(b)||[];e.set(b,f);pj(e,d,a);dj.delete(b)}else e=new Map,f=cj.get(b)||[],e.set(b,f),nj(e,d,a),cj.delete(b);else c?(pj(dj,d,a),dj.clear()):(nj(cj,d,a),cj.clear());else lj(c),d()})}
function lj(a){a=void 0===a?!1:a;if(K("web_gel_timeout_cap")&&(!a&&!$i.h||a&&!aj.h)){var b=Rh(function(){kj({writeThenSend:!0},void 0,a)},6E4);
a?aj.h=b:$i.h=b}Sh(a?aj.i:$i.i);b=B("LOGGING_BATCH_TIMEOUT",xh("web_gel_debounce_ms",1E4));K("shorten_initial_gel_batch_timeout")&&bj&&(b=Wi);b=Rh(function(){kj({writeThenSend:!0},void 0,a)},b);
a?aj.i=b:$i.i=b}
function nj(a,b,c,d){var e=Yi;c=void 0===c?{}:c;var f=Math.round(O()),g=a.size;a=q(a);for(var h=a.next();!h.done;h=a.next()){var k=q(h.value);h=k.next().value;var l=k=k.next().value;k=vb({context:qj(e.config_||rj())});k.events=l;(l=ej[h])&&sj(k,h,l);delete ej[h];h="visitorOnlyApprovedKey"===h;tj(k,f,h);uj(c);vj(e,"log_event",k,wj(c,h,function(){g--;g||b()},function(){g--;
g||b()},d));
bj=!1}}
function pj(a,b,c,d){var e=Yi;c=void 0===c?{}:c;var f=Math.round(O()),g=a.size;a=q(a);for(var h=a.next();!h.done;h=a.next()){var k=q(h.value);h=k.next().value;var l=k=k.next().value;k=new dh;var n=xj(e.config_||rj());G(k,1,n);l=yj(l);for(n=0;n<l.length;n++)Xc(k,3,$g,l[n]);(l=fj[h])&&zj(k,h,l);delete fj[h];h="visitorOnlyApprovedKey"===h;Aj(k,f,h);uj(c);k=gd(k);h=wj(c,h,function(){g--;g||b()},function(){g--;
g||b()},d);
h.headers={"Content-Type":"application/json+protobuf"};h.postBodyFormat="JSPB";h.postBody=k;vj(e,"log_event","",h);bj=!1}}
function uj(a){K("always_send_and_write")&&(a.writeThenSend=!1)}
function wj(a,b,c,d,e){return{retry:!0,onSuccess:c,onError:d,Bb:a,wa:b,wo:!!e,headers:{},postBodyFormat:"",postBody:""}}
function tj(a,b,c){a.requestTimeMs=String(b);K("unsplit_gel_payloads_in_logs")&&(a.unsplitGelPayloadsInLogs=!0);!c&&(b=B("EVENT_ID",void 0))&&(c=Bj(),a.serializedClientEventId={serializedEventId:b,clientCounter:String(c)})}
function Aj(a,b,c){F(a,2,b);if(!c&&(b=B("EVENT_ID",void 0))){c=Bj();var d=new ah;F(d,1,b);F(d,2,c);G(a,5,d)}}
function Bj(){var a=B("BATCH_CLIENT_COUNTER",void 0)||0;a||(a=Math.floor(Math.random()*Xi/2));a++;a>Xi&&(a=1);uh("BATCH_CLIENT_COUNTER",a);return a}
function sj(a,b,c){if(c.videoId)var d="VIDEO";else if(c.playlistId)d="PLAYLIST";else return;a.credentialTransferTokenTargetId=c;a.context=a.context||{};a.context.user=a.context.user||{};a.context.user.credentialTransferTokens=[{token:b,scope:d}]}
function zj(a,b,c){if(Pc(c,1===Sc(c,ch)?1:-1))var d=1;else if(c.getPlaylistId())d=2;else return;G(a,4,c);a=Tc(a,Dg,1)||new Dg;c=Tc(a,Bg,3)||new Bg;var e=new Ag;e.setToken(b);F(e,1,d);Xc(c,12,Ag,e);G(a,3,c)}
function yj(a){for(var b=[],c=0;c<a.length;c++)try{var d=b,e=d.push,f=String(a[c]);var g=ed($g,f?JSON.parse(f):null);e.call(d,g)}catch(h){M(new N("Transport failed to deserialize "+String(a[c])))}return b}
;var Cj=x.ytLoggingGelSequenceIdObj_||{};z("ytLoggingGelSequenceIdObj_",Cj,void 0);
function Dj(a,b,c,d){d=void 0===d?{}:d;if(K("lr_drop_other_and_business_payloads")){if(Bh[a]||Ah[a])return}else if(K("lr_drop_other_payloads")&&Bh[a])return;var e={},f=Math.round(d.timestamp||O());e.eventTimeMs=f<Number.MAX_SAFE_INTEGER?f:0;e[a]=b;a=fi();e.context={lastActivityMs:String(d.timestamp||!isFinite(a)?-1:a)};K("log_sequence_info_on_gel_web")&&d.X&&(a=e.context,b=d.X,b={index:Ej(b),groupKey:b},a.sequence=b,d.ub&&delete Cj[d.X]);(d.mc?mj:gj)({endpoint:"log_event",payload:e,cttAuthInfo:d.cttAuthInfo,
wa:d.wa},c)}
function Ej(a){Cj[a]=a in Cj?Cj[a]+1:0;return Cj[a]}
;function Fj(a){var b=this;this.h=void 0;a.addEventListener("beforeinstallprompt",function(c){c.preventDefault();b.h=c})}
function Gj(){if(!x.matchMedia)return"WEB_DISPLAY_MODE_UNKNOWN";try{return x.matchMedia("(display-mode: standalone)").matches?"WEB_DISPLAY_MODE_STANDALONE":x.matchMedia("(display-mode: minimal-ui)").matches?"WEB_DISPLAY_MODE_MINIMAL_UI":x.matchMedia("(display-mode: fullscreen)").matches?"WEB_DISPLAY_MODE_FULLSCREEN":x.matchMedia("(display-mode: browser)").matches?"WEB_DISPLAY_MODE_BROWSER":"WEB_DISPLAY_MODE_UNKNOWN"}catch(a){return"WEB_DISPLAY_MODE_UNKNOWN"}}
function Hj(){var a=Gj();a=Object.keys(Ne).indexOf(a);return-1===a?null:a}
;function Ij(a,b,c,d,e){Wd.set(""+a,b,{Pa:c,path:"/",domain:void 0===d?"youtube.com":d,secure:void 0===e?!1:e})}
function Jj(a){return Wd.get(""+a,void 0)}
function Kj(){if(!Wd.isEnabled())return!1;if(!Wd.isEmpty())return!0;Wd.set("TESTCOOKIESENABLED","1",{Pa:60});if("1"!==Wd.get("TESTCOOKIESENABLED"))return!1;Wd.remove("TESTCOOKIESENABLED");return!0}
;var Lj=A("ytglobal.prefsUserPrefsPrefs_")||{};z("ytglobal.prefsUserPrefsPrefs_",Lj,void 0);function Mj(){this.h=B("ALT_PREF_COOKIE_NAME","PREF");this.i=B("ALT_PREF_COOKIE_DOMAIN","youtube.com");var a=Jj(this.h);if(a){a=decodeURIComponent(a).split("&");for(var b=0;b<a.length;b++){var c=a[b].split("="),d=c[0];(c=c[1])&&(Lj[d]=c.toString())}}}
Mj.prototype.get=function(a,b){Nj(a);Oj(a);a=void 0!==Lj[a]?Lj[a].toString():null;return null!=a?a:b?b:""};
Mj.prototype.set=function(a,b){Nj(a);Oj(a);if(null==b)throw Error("ExpectedNotNull");Lj[a]=b.toString()};
function Pj(a){return!!((Qj("f"+(Math.floor(a/31)+1))||0)&1<<a%31)}
Mj.prototype.remove=function(a){Nj(a);Oj(a);delete Lj[a]};
Mj.prototype.clear=function(){for(var a in Lj)delete Lj[a]};
function Oj(a){if(/^f([1-9][0-9]*)$/.test(a))throw Error("ExpectedRegexMatch: "+a);}
function Nj(a){if(!/^\w+$/.test(a))throw Error("ExpectedRegexMismatch: "+a);}
function Qj(a){a=void 0!==Lj[a]?Lj[a].toString():null;return null!=a&&/^[A-Fa-f0-9]+$/.test(a)?parseInt(a,16):null}
Ma(Mj);var Rj={bluetooth:"CONN_DISCO",cellular:"CONN_CELLULAR_UNKNOWN",ethernet:"CONN_WIFI",none:"CONN_NONE",wifi:"CONN_WIFI",wimax:"CONN_CELLULAR_4G",other:"CONN_UNKNOWN",unknown:"CONN_UNKNOWN","slow-2g":"CONN_CELLULAR_2G","2g":"CONN_CELLULAR_2G","3g":"CONN_CELLULAR_3G","4g":"CONN_CELLULAR_4G"},Sj={CONN_DEFAULT:0,CONN_UNKNOWN:1,CONN_NONE:2,CONN_WIFI:3,CONN_CELLULAR_2G:4,CONN_CELLULAR_3G:5,CONN_CELLULAR_4G:6,CONN_CELLULAR_UNKNOWN:7,CONN_DISCO:8,CONN_CELLULAR_5G:9,CONN_WIFI_METERED:10,CONN_CELLULAR_5G_SA:11,
CONN_CELLULAR_5G_NSA:12,CONN_INVALID:31},Tj={EFFECTIVE_CONNECTION_TYPE_UNKNOWN:0,EFFECTIVE_CONNECTION_TYPE_OFFLINE:1,EFFECTIVE_CONNECTION_TYPE_SLOW_2G:2,EFFECTIVE_CONNECTION_TYPE_2G:3,EFFECTIVE_CONNECTION_TYPE_3G:4,EFFECTIVE_CONNECTION_TYPE_4G:5},Uj={"slow-2g":"EFFECTIVE_CONNECTION_TYPE_SLOW_2G","2g":"EFFECTIVE_CONNECTION_TYPE_2G","3g":"EFFECTIVE_CONNECTION_TYPE_3G","4g":"EFFECTIVE_CONNECTION_TYPE_4G"};function Vj(){var a=x.navigator;return a?a.connection:void 0}
function Wj(){var a=Vj();if(a){var b=Rj[a.type||"unknown"]||"CONN_UNKNOWN";a=Rj[a.effectiveType||"unknown"]||"CONN_UNKNOWN";"CONN_CELLULAR_UNKNOWN"===b&&"CONN_UNKNOWN"!==a&&(b=a);if("CONN_UNKNOWN"!==b)return b;if("CONN_UNKNOWN"!==a)return a}}
function Xj(){var a=Vj();if(null!==a&&void 0!==a&&a.effectiveType)return Uj.hasOwnProperty(a.effectiveType)?Uj[a.effectiveType]:"EFFECTIVE_CONNECTION_TYPE_UNKNOWN"}
;function Yj(){return"INNERTUBE_API_KEY"in oh&&"INNERTUBE_API_VERSION"in oh}
function rj(){return{innertubeApiKey:B("INNERTUBE_API_KEY",void 0),innertubeApiVersion:B("INNERTUBE_API_VERSION",void 0),bb:B("INNERTUBE_CONTEXT_CLIENT_CONFIG_INFO"),cb:B("INNERTUBE_CONTEXT_CLIENT_NAME","WEB"),Vb:B("INNERTUBE_CONTEXT_CLIENT_NAME",1),innertubeContextClientVersion:B("INNERTUBE_CONTEXT_CLIENT_VERSION",void 0),yb:B("INNERTUBE_CONTEXT_HL",void 0),xb:B("INNERTUBE_CONTEXT_GL",void 0),Wb:B("INNERTUBE_HOST_OVERRIDE",void 0)||"",Yb:!!B("INNERTUBE_USE_THIRD_PARTY_AUTH",!1),Xb:!!B("INNERTUBE_OMIT_API_KEY_WHEN_AUTH_HEADER_IS_PRESENT",
!1),appInstallData:B("SERIALIZED_CLIENT_CONFIG_DATA",void 0)}}
function qj(a){var b={client:{hl:a.yb,gl:a.xb,clientName:a.cb,clientVersion:a.innertubeContextClientVersion,configInfo:a.bb}};navigator.userAgent&&(b.client.userAgent=String(navigator.userAgent));var c=x.devicePixelRatio;c&&1!=c&&(b.client.screenDensityFloat=String(c));c=yh();""!==c&&(b.client.experimentsToken=c);c=zh();0<c.length&&(b.request={internalExperimentFlags:c});Zj(a,void 0,b);ak(a,void 0,b);bk(void 0,b);ck(a,void 0,b);dk(void 0,b);B("DELEGATED_SESSION_ID")&&!K("pageid_as_header_web")&&(b.user=
{onBehalfOfUser:B("DELEGATED_SESSION_ID")});a=Object;c=a.assign;for(var d=b.client,e={},f=q(Object.entries(pi(B("DEVICE","")))),g=f.next();!g.done;g=f.next()){var h=q(g.value);g=h.next().value;h=h.next().value;"cbrand"===g?e.deviceMake=h:"cmodel"===g?e.deviceModel=h:"cbr"===g?e.browserName=h:"cbrver"===g?e.browserVersion=h:"cos"===g?e.osName=h:"cosver"===g?e.osVersion=h:"cplatform"===g&&(e.platform=h)}b.client=c.call(a,d,e);return b}
function xj(a){var b=new Dg,c=new wg;F(c,1,a.yb);F(c,2,a.xb);F(c,16,a.Vb);F(c,17,a.innertubeContextClientVersion);if(a.bb){var d=a.bb,e=new tg;d.coldConfigData&&F(e,1,d.coldConfigData);d.appInstallData&&F(e,6,d.appInstallData);d.coldHashData&&F(e,3,d.coldHashData);d.hotHashData&&F(e,5,d.hotHashData);G(c,62,e)}(d=x.devicePixelRatio)&&1!=d&&F(c,65,d);d=yh();""!==d&&F(c,54,d);d=zh();if(0<d.length){e=new yg;for(var f=0;f<d.length;f++){var g=new rg;F(g,1,d[f].key);g.setValue(d[f].value);Xc(e,15,rg,g)}G(b,
5,e)}Zj(a,c);ak(a,c);bk(c);ck(a,c);dk(c);B("DELEGATED_SESSION_ID")&&!K("pageid_as_header_web")&&(a=new Bg,F(a,3,B("DELEGATED_SESSION_ID")));a=q(Object.entries(pi(B("DEVICE",""))));for(d=a.next();!d.done;d=a.next())e=q(d.value),d=e.next().value,e=e.next().value,"cbrand"===d?F(c,12,e):"cmodel"===d?F(c,13,e):"cbr"===d?F(c,87,e):"cbrver"===d?F(c,88,e):"cos"===d?F(c,18,e):"cosver"===d?F(c,19,e):"cplatform"===d&&F(c,42,e);G(b,1,c);return b}
function Zj(a,b,c){a=a.cb;if("WEB"===a||"MWEB"===a||1===a||2===a)if(b){c=Tc(b,ug,96)||new ug;var d=Hj();null!==d&&F(c,3,d);G(b,96,c)}else c&&(c.client.mainAppWebInfo=null!=(d=c.client.mainAppWebInfo)?d:{},c.client.mainAppWebInfo.webDisplayMode=Gj())}
function ak(a,b,c){a=a.cb;if(("WEB_REMIX"===a||76===a)&&!K("music_web_display_mode_killswitch"))if(b){var d;c=null!=(d=Tc(b,vg,70))?d:new vg;d=Hj();null!==d&&F(c,10,d);G(b,70,c)}else if(c){var e;c.client.Ab=null!=(e=c.client.Ab)?e:{};c.client.Ab.webDisplayMode=Gj()}}
function bk(a,b){var c;if(K("web_log_memory_total_kbytes")&&(null==(c=x.navigator)?0:c.deviceMemory)){var d;c=null==(d=x.navigator)?void 0:d.deviceMemory;a?F(a,95,1E6*c):b&&(b.client.memoryTotalKbytes=""+1E6*c)}}
function ck(a,b,c){if(a.appInstallData)if(b){var d;c=null!=(d=Tc(b,tg,62))?d:new tg;F(c,6,a.appInstallData);G(b,62,c)}else c&&(c.client.configInfo=c.client.configInfo||{},c.client.configInfo.appInstallData=a.appInstallData)}
function dk(a,b){var c=Wj();c&&(a?F(a,61,Sj[c]):b&&(b.client.connectionType=c));K("web_log_effective_connection_type")&&(c=Xj())&&(a?F(a,94,Tj[c]):b&&(b.client.effectiveConnectionType=c))}
function ek(a,b,c){c=void 0===c?{}:c;var d={};K("enable_web_eom_visitor_data")&&B("EOM_VISITOR_DATA")?d={"X-Goog-EOM-Visitor-Id":B("EOM_VISITOR_DATA")}:d={"X-Goog-Visitor-Id":c.visitorData||B("VISITOR_DATA","")};if(b&&b.includes("www.youtube-nocookie.com"))return d;(b=c.uo||B("AUTHORIZATION"))||(a?b="Bearer "+A("gapi.auth.getToken")().to:b=$d([]));b&&(d.Authorization=b,d["X-Goog-AuthUser"]=B("SESSION_INDEX",0),K("pageid_as_header_web")&&(d["X-Goog-PageId"]=B("DELEGATED_SESSION_ID")));return d}
;function fk(a){a=Object.assign({},a);delete a.Authorization;var b=$d();if(b){var c=new ff;c.update(B("INNERTUBE_API_KEY",void 0));c.update(b);a.hash=Bc(c.digest(),3)}return a}
;function gk(a){var b=new kg;(b=b.isAvailable()?a?new qg(b,a):b:null)||(a=new lg(a||"UserDataSharedStore"),b=a.isAvailable()?a:null);this.h=(a=b)?new gg(a):null;this.i=document.domain||window.location.hostname}
gk.prototype.set=function(a,b,c,d){c=c||31104E3;this.remove(a);if(this.h)try{this.h.set(a,b,Date.now()+1E3*c);return}catch(f){}var e="";if(d)try{e=escape(Cf(b))}catch(f){return}else e=escape(b);Ij(a,e,c,this.i)};
gk.prototype.get=function(a,b){var c=void 0,d=!this.h;if(!d)try{c=this.h.get(a)}catch(e){d=!0}if(d&&(c=Jj(a))&&(c=unescape(c),b))try{c=JSON.parse(c)}catch(e){this.remove(a),c=void 0}return c};
gk.prototype.remove=function(a){this.h&&this.h.remove(a);var b=this.i;Wd.remove(""+a,"/",void 0===b?"youtube.com":b)};var hk;function ik(){hk||(hk=new gk("yt.innertube"));return hk}
function jk(a,b,c,d){if(d)return null;d=ik().get("nextId",!0)||1;var e=ik().get("requests",!0)||{};e[d]={method:a,request:b,authState:fk(c),requestTime:Math.round(O())};ik().set("nextId",d+1,86400,!0);ik().set("requests",e,86400,!0);return d}
function kk(a){var b=ik().get("requests",!0)||{};delete b[a];ik().set("requests",b,86400,!0)}
function lk(a){var b=ik().get("requests",!0);if(b){for(var c in b){var d=b[c];if(!(6E4>Math.round(O())-d.requestTime)){var e=d.authState,f=fk(ek(!1));tb(e,f)&&(e=d.request,"requestTimeMs"in e&&(e.requestTimeMs=Math.round(O())),vj(a,d.method,e,{}));delete b[c]}}ik().set("requests",b,86400,!0)}}
;var mk=wc||xc;function nk(a){var b=Ob();return b?0<=b.toLowerCase().indexOf(a):!1}
;var ok=function(){var a;return function(){a||(a=new gk("ytidb"));return a}}();
function pk(){var a;return null===(a=ok())||void 0===a?void 0:a.get("LAST_RESULT_ENTRY_KEY",!0)}
;var qk=[],rk,sk=!1;function tk(){var a={};for(rk=new uk(void 0===a.handleError?vk:a.handleError,void 0===a.logEvent?wk:a.logEvent);0<qk.length;)switch(a=qk.shift(),a.type){case "ERROR":rk.handleError(a.payload);break;case "EVENT":rk.logEvent(a.eventType,a.payload)}}
function xk(a){sk||(rk?rk.handleError(a):(qk.push({type:"ERROR",payload:a}),10<qk.length&&qk.shift()))}
function yk(a,b){sk||(rk?rk.logEvent(a,b):(qk.push({type:"EVENT",eventType:a,payload:b}),10<qk.length&&qk.shift()))}
;function zk(){try{return Ak(),!0}catch(a){return!1}}
function Ak(a){if(void 0!==B("DATASYNC_ID",void 0))return B("DATASYNC_ID",void 0);throw new N("Datasync ID not set",void 0===a?"unknown":a);}
;function Bk(a){if(0<=a.indexOf(":"))throw Error("Database name cannot contain ':'");}
function Ck(a){return a.substr(0,a.indexOf(":"))||a}
;var Dk={},Ek=(Dk.AUTH_INVALID="No user identifier specified.",Dk.EXPLICIT_ABORT="Transaction was explicitly aborted.",Dk.IDB_NOT_SUPPORTED="IndexedDB is not supported.",Dk.MISSING_INDEX="Index not created.",Dk.MISSING_OBJECT_STORES="Object stores not created.",Dk.DB_DELETED_BY_MISSING_OBJECT_STORES="Database is deleted because expected object stores were not created.",Dk.DB_REOPENED_BY_MISSING_OBJECT_STORES="Database is reopened because expected object stores were not created.",Dk.UNKNOWN_ABORT="Transaction was aborted for unknown reasons.",
Dk.QUOTA_EXCEEDED="The current transaction exceeded its quota limitations.",Dk.QUOTA_MAYBE_EXCEEDED="The current transaction may have failed because of exceeding quota limitations.",Dk.EXECUTE_TRANSACTION_ON_CLOSED_DB="Can't start a transaction on a closed database",Dk.INCOMPATIBLE_DB_VERSION="The binary is incompatible with the database version",Dk),Fk={},Gk=(Fk.AUTH_INVALID="ERROR",Fk.EXECUTE_TRANSACTION_ON_CLOSED_DB="WARNING",Fk.EXPLICIT_ABORT="IGNORED",Fk.IDB_NOT_SUPPORTED="ERROR",Fk.MISSING_INDEX=
"WARNING",Fk.MISSING_OBJECT_STORES="ERROR",Fk.DB_DELETED_BY_MISSING_OBJECT_STORES="WARNING",Fk.DB_REOPENED_BY_MISSING_OBJECT_STORES="WARNING",Fk.QUOTA_EXCEEDED="WARNING",Fk.QUOTA_MAYBE_EXCEEDED="WARNING",Fk.UNKNOWN_ABORT="WARNING",Fk.INCOMPATIBLE_DB_VERSION="WARNING",Fk),Hk={},Ik=(Hk.AUTH_INVALID=!1,Hk.EXECUTE_TRANSACTION_ON_CLOSED_DB=!1,Hk.EXPLICIT_ABORT=!1,Hk.IDB_NOT_SUPPORTED=!1,Hk.MISSING_INDEX=!1,Hk.MISSING_OBJECT_STORES=!1,Hk.DB_DELETED_BY_MISSING_OBJECT_STORES=!1,Hk.DB_REOPENED_BY_MISSING_OBJECT_STORES=
!1,Hk.QUOTA_EXCEEDED=!1,Hk.QUOTA_MAYBE_EXCEEDED=!0,Hk.UNKNOWN_ABORT=!0,Hk.INCOMPATIBLE_DB_VERSION=!1,Hk);function Jk(a,b,c,d,e){b=void 0===b?{}:b;c=void 0===c?Ek[a]:c;d=void 0===d?Gk[a]:d;e=void 0===e?Ik[a]:e;N.call(this,c,Object.assign({name:"YtIdbKnownError",isSw:void 0===self.document,isIframe:self!==self.top,type:a},b));this.type=a;this.message=c;this.level=d;this.h=e;Object.setPrototypeOf(this,Jk.prototype)}
r(Jk,N);function Kk(a,b){Jk.call(this,"MISSING_OBJECT_STORES",{expectedObjectStores:b,foundObjectStores:a},Ek.MISSING_OBJECT_STORES);Object.setPrototypeOf(this,Kk.prototype)}
r(Kk,Jk);function Lk(a,b){var c=Error.call(this);this.message=c.message;"stack"in c&&(this.stack=c.stack);this.index=a;this.objectStore=b;Object.setPrototypeOf(this,Lk.prototype)}
r(Lk,Error);var Mk=["The database connection is closing","Can't start a transaction on a closed database","A mutation operation was attempted on a database that did not allow mutations"];
function Nk(a,b,c,d){b=Ck(b);var e=a instanceof Error?a:Error("Unexpected error: "+a);if(e instanceof Jk)return e;a={objectStoreNames:c,dbName:b,dbVersion:d};if("QuotaExceededError"===e.name)return new Jk("QUOTA_EXCEEDED",a);if(yc&&"UnknownError"===e.name)return new Jk("QUOTA_MAYBE_EXCEEDED",a);if(e instanceof Lk)return new Jk("MISSING_INDEX",Object.assign(Object.assign({},a),{objectStore:e.objectStore,index:e.index}));if("InvalidStateError"===e.name&&Mk.some(function(f){return e.message.includes(f)}))return new Jk("EXECUTE_TRANSACTION_ON_CLOSED_DB",
a);
if("AbortError"===e.name)return new Jk("UNKNOWN_ABORT",a,e.message);e.args=[Object.assign(Object.assign({},a),{name:"IdbError",Cb:e.name})];e.level="WARNING";return e}
function Ok(a,b,c){var d=pk();return new Jk("IDB_NOT_SUPPORTED",{context:{caller:a,publicName:b,version:c,hasSucceededOnce:null===d||void 0===d?void 0:d.hasSucceededOnce}})}
;function Pk(a){if(!a)throw Error();throw a;}
function Qk(a){return a}
function Rk(a){this.h=a}
function Sk(a){function b(e){if("PENDING"===d.state.status){d.state={status:"REJECTED",reason:e};e=q(d.onRejected);for(var f=e.next();!f.done;f=e.next())f=f.value,f()}}
function c(e){if("PENDING"===d.state.status){d.state={status:"FULFILLED",value:e};e=q(d.h);for(var f=e.next();!f.done;f=e.next())f=f.value,f()}}
var d=this;this.state={status:"PENDING"};this.h=[];this.onRejected=[];a=a.h;try{a(c,b)}catch(e){b(e)}}
Sk.all=function(a){return new Sk(new Rk(function(b,c){var d=[],e=a.length;0===e&&b(d);for(var f={na:0};f.na<a.length;f={na:f.na},++f.na)Tk(Sk.resolve(a[f.na]).then(function(g){return function(h){d[g.na]=h;e--;0===e&&b(d)}}(f)),function(g){c(g)})}))};
Sk.resolve=function(a){return new Sk(new Rk(function(b,c){a instanceof Sk?a.then(b,c):b(a)}))};
Sk.reject=function(a){return new Sk(new Rk(function(b,c){c(a)}))};
Sk.prototype.then=function(a,b){var c=this,d=null!==a&&void 0!==a?a:Qk,e=null!==b&&void 0!==b?b:Pk;return new Sk(new Rk(function(f,g){"PENDING"===c.state.status?(c.h.push(function(){Uk(c,c,d,f,g)}),c.onRejected.push(function(){Vk(c,c,e,f,g)})):"FULFILLED"===c.state.status?Uk(c,c,d,f,g):"REJECTED"===c.state.status&&Vk(c,c,e,f,g)}))};
function Tk(a,b){a.then(void 0,b)}
function Uk(a,b,c,d,e){try{if("FULFILLED"!==a.state.status)throw Error("calling handleResolve before the promise is fulfilled.");var f=c(a.state.value);f instanceof Sk?Wk(a,b,f,d,e):d(f)}catch(g){e(g)}}
function Vk(a,b,c,d,e){try{if("REJECTED"!==a.state.status)throw Error("calling handleReject before the promise is rejected.");var f=c(a.state.reason);f instanceof Sk?Wk(a,b,f,d,e):d(f)}catch(g){e(g)}}
function Wk(a,b,c,d,e){b===c?e(new TypeError("Circular promise chain detected.")):c.then(function(f){f instanceof Sk?Wk(a,b,f,d,e):d(f)},function(f){e(f)})}
;function Xk(a,b,c){function d(){c(a.error);f()}
function e(){b(a.result);f()}
function f(){try{a.removeEventListener("success",e),a.removeEventListener("error",d)}catch(g){}}
a.addEventListener("success",e);a.addEventListener("error",d)}
function Yk(a){return new Promise(function(b,c){Xk(a,b,c)})}
function Zk(a){return new Sk(new Rk(function(b,c){Xk(a,b,c)}))}
;function $k(a,b){return new Sk(new Rk(function(c,d){function e(){var f=a?b(a):null;f?f.then(function(g){a=g;e()},d):c()}
e()}))}
;function al(a,b){this.h=a;this.options=b;this.transactionCount=0;this.j=Math.round(O());this.i=!1}
m=al.prototype;m.add=function(a,b,c){return bl(this,[a],{mode:"readwrite",O:!0},function(d){return d.objectStore(a).add(b,c)})};
m.clear=function(a){return bl(this,[a],{mode:"readwrite",O:!0},function(b){return b.objectStore(a).clear()})};
m.close=function(){var a;this.h.close();(null===(a=this.options)||void 0===a?0:a.closed)&&this.options.closed()};
m.count=function(a,b){return bl(this,[a],{mode:"readonly",O:!0},function(c){return c.objectStore(a).count(b)})};
function cl(a,b,c){a=a.h.createObjectStore(b,c);return new dl(a)}
m.delete=function(a,b){return bl(this,[a],{mode:"readwrite",O:!0},function(c){return c.objectStore(a).delete(b)})};
m.get=function(a,b){return bl(this,[a],{mode:"readonly",O:!0},function(c){return c.objectStore(a).get(b)})};
function el(a,b){return bl(a,["LogsRequestsStore"],{mode:"readwrite",O:!0},function(c){c=c.objectStore("LogsRequestsStore");return Zk(c.h.put(b,void 0))})}
m.objectStoreNames=function(){return Array.from(this.h.objectStoreNames)};
function bl(a,b,c,d){var e,f,g,h,k,l,n,t,y,u,D,E;return w(function(L){switch(L.h){case 1:var P={mode:"readonly",O:!1,tag:"IDB_TRANSACTION_TAG_UNKNOWN"};"string"===typeof c?P.mode=c:Object.assign(P,c);e=P;a.transactionCount++;f=e.O?3:1;g=0;case 2:if(h){L.s(3);break}g++;k=Math.round(O());ua(L,4);l=a.h.transaction(b,e.mode);P=new fl(l);P=gl(P,d);return v(L,P,6);case 6:return n=L.i,t=Math.round(O()),hl(a,k,t,g,void 0,b.join(),e),L.return(n);case 4:y=wa(L);u=Math.round(O());D=Nk(y,a.h.name,b.join(),a.h.version);
if((E=D instanceof Jk&&!D.h)||g>=f)hl(a,k,u,g,D,b.join(),e),h=D;L.s(2);break;case 3:return L.return(Promise.reject(h))}})}
function hl(a,b,c,d,e,f,g){b=c-b;e?(e instanceof Jk&&("QUOTA_EXCEEDED"===e.type||"QUOTA_MAYBE_EXCEEDED"===e.type)&&yk("QUOTA_EXCEEDED",{dbName:Ck(a.h.name),objectStoreNames:f,transactionCount:a.transactionCount,transactionMode:g.mode}),e instanceof Jk&&"UNKNOWN_ABORT"===e.type&&(c-=a.j,0>c&&c>=Math.pow(2,31)&&(c=0),yk("TRANSACTION_UNEXPECTEDLY_ABORTED",{objectStoreNames:f,transactionDuration:b,transactionCount:a.transactionCount,dbDuration:c}),a.i=!0),il(a,!1,d,f,b,g.tag),xk(e)):il(a,!0,d,f,b,g.tag)}
function il(a,b,c,d,e,f){yk("TRANSACTION_ENDED",{objectStoreNames:d,connectionHasUnknownAbortedTransaction:a.i,duration:e,isSuccessful:b,tryCount:c,tag:void 0===f?"IDB_TRANSACTION_TAG_UNKNOWN":f})}
m.getName=function(){return this.h.name};
function dl(a){this.h=a}
m=dl.prototype;m.add=function(a,b){return Zk(this.h.add(a,b))};
m.autoIncrement=function(){return this.h.autoIncrement};
m.clear=function(){return Zk(this.h.clear()).then(function(){})};
m.count=function(a){return Zk(this.h.count(a))};
function jl(a,b){return kl(a,{query:b},function(c){return c.delete().then(function(){return c.continue()})}).then(function(){})}
m.delete=function(a){return a instanceof IDBKeyRange?jl(this,a):Zk(this.h.delete(a))};
m.get=function(a){return Zk(this.h.get(a))};
m.index=function(a){try{return new ll(this.h.index(a))}catch(b){if(b instanceof Error&&"NotFoundError"===b.name)throw new Lk(a,this.h.name);throw b;}};
m.getName=function(){return this.h.name};
m.keyPath=function(){return this.h.keyPath};
function kl(a,b,c){a=a.h.openCursor(b.query,b.direction);return ml(a).then(function(d){return $k(d,c)})}
function fl(a){var b=this;this.h=a;this.j=new Map;this.i=!1;this.done=new Promise(function(c,d){b.h.addEventListener("complete",function(){c()});
b.h.addEventListener("error",function(e){e.currentTarget===e.target&&d(b.h.error)});
b.h.addEventListener("abort",function(){var e=b.h.error;if(e)d(e);else if(!b.i){e=Jk;for(var f=b.h.objectStoreNames,g=[],h=0;h<f.length;h++){var k=f.item(h);if(null===k)throw Error("Invariant: item in DOMStringList is null");g.push(k)}e=new e("UNKNOWN_ABORT",{objectStoreNames:g.join(),dbName:b.h.db.name,mode:b.h.mode});d(e)}})})}
function gl(a,b){var c=new Promise(function(d,e){try{Tk(b(a).then(function(f){d(f)}),e)}catch(f){e(f),a.abort()}});
return Promise.all([c,a.done]).then(function(d){return q(d).next().value})}
fl.prototype.abort=function(){this.h.abort();this.i=!0;throw new Jk("EXPLICIT_ABORT");};
fl.prototype.objectStore=function(a){a=this.h.objectStore(a);var b=this.j.get(a);b||(b=new dl(a),this.j.set(a,b));return b};
function ll(a){this.h=a}
m=ll.prototype;m.count=function(a){return Zk(this.h.count(a))};
m.delete=function(a){return nl(this,{query:a},function(b){return b.delete().then(function(){return b.continue()})})};
m.get=function(a){return Zk(this.h.get(a))};
m.getKey=function(a){return Zk(this.h.getKey(a))};
m.keyPath=function(){return this.h.keyPath};
m.unique=function(){return this.h.unique};
function nl(a,b,c){a=a.h.openCursor(void 0===b.query?null:b.query,void 0===b.direction?"next":b.direction);return ml(a).then(function(d){return $k(d,c)})}
function ol(a,b){this.request=a;this.cursor=b}
function ml(a){return Zk(a).then(function(b){return b?new ol(a,b):null})}
m=ol.prototype;m.advance=function(a){this.cursor.advance(a);return ml(this.request)};
m.continue=function(a){this.cursor.continue(a);return ml(this.request)};
m.delete=function(){return Zk(this.cursor.delete()).then(function(){})};
m.getKey=function(){return this.cursor.key};
m.getValue=function(){return this.cursor.value};
m.update=function(a){return Zk(this.cursor.update(a))};function pl(a,b,c){return new Promise(function(d,e){function f(){y||(y=new al(g.result,{closed:t}));return y}
var g=void 0!==b?self.indexedDB.open(a,b):self.indexedDB.open(a);var h=c.blocked,k=c.blocking,l=c.yc,n=c.upgrade,t=c.closed,y;g.addEventListener("upgradeneeded",function(u){try{if(null===u.newVersion)throw Error("Invariant: newVersion on IDbVersionChangeEvent is null");if(null===g.transaction)throw Error("Invariant: transaction on IDbOpenDbRequest is null");u.dataLoss&&"none"!==u.dataLoss&&yk("IDB_DATA_CORRUPTED",{reason:u.dataLossMessage||"unknown reason",dbName:Ck(a)});var D=f(),E=new fl(g.transaction);
n&&n(D,function(L){return u.oldVersion<L&&u.newVersion>=L},E);
E.done.catch(function(L){e(L)})}catch(L){e(L)}});
g.addEventListener("success",function(){var u=g.result;k&&u.addEventListener("versionchange",function(){k(f())});
u.addEventListener("close",function(){yk("IDB_UNEXPECTEDLY_CLOSED",{dbName:Ck(a),dbVersion:u.version});l&&l()});
d(f())});
g.addEventListener("error",function(){e(g.error)});
h&&g.addEventListener("blocked",function(){h()})})}
function ql(a,b,c){c=void 0===c?{}:c;return pl(a,b,c)}
function rl(a,b){b=void 0===b?{}:b;var c,d,e,f;return w(function(g){if(1==g.h)return ua(g,2),c=self.indexedDB.deleteDatabase(a),d=b,(e=d.blocked)&&c.addEventListener("blocked",function(){e()}),v(g,Yk(c),4);
if(2!=g.h)return va(g,0);f=wa(g);throw Nk(f,a,"",-1);})}
;function sl(a){return new Promise(function(b){Xh(function(){b()},a)})}
function tl(a,b){this.name=a;this.options=b;this.l=!0;this.m=this.o=0;this.i=500}
tl.prototype.j=function(a,b,c){c=void 0===c?{}:c;return ql(a,b,c)};
tl.prototype.delete=function(a){a=void 0===a?{}:a;return rl(this.name,a)};
function ul(a,b){return new Jk("INCOMPATIBLE_DB_VERSION",{dbName:a.name,oldVersion:a.options.version,newVersion:b})}
function vl(a,b){if(!b)throw Ok("openWithToken",Ck(a.name));return a.open()}
tl.prototype.open=function(){function a(){var f,g,h,k,l,n,t,y,u,D;return w(function(E){switch(E.h){case 1:return h=null!==(f=Error().stack)&&void 0!==f?f:"",ua(E,2),v(E,c.j(c.name,c.options.version,e),4);case 4:k=E.i;for(var L=c.options,P=[],R=q(Object.keys(L.ya)),V=R.next();!V.done;V=R.next()){V=V.value;var aa=L.ya[V],Cd=void 0===aa.hc?Number.MAX_VALUE:aa.hc;!(k.h.version>=aa.Wa)||k.h.version>=Cd||k.h.objectStoreNames.contains(V)||P.push(V)}l=P;if(0===l.length){E.s(5);break}n=Object.keys(c.options.ya);
t=k.objectStoreNames();if(c.m<xh("ytidb_reopen_db_retries",0))return c.m++,k.close(),xk(new Jk("DB_REOPENED_BY_MISSING_OBJECT_STORES",{dbName:c.name,expectedObjectStores:n,foundObjectStores:t})),E.return(a());if(!(c.o<xh("ytidb_remake_db_retries",1))){E.s(6);break}c.o++;if(!K("ytidb_remake_db_enable_backoff_delay")){E.s(7);break}return v(E,sl(c.i),8);case 8:c.i*=2;case 7:return v(E,c.delete(),9);case 9:return xk(new Jk("DB_DELETED_BY_MISSING_OBJECT_STORES",{dbName:c.name,expectedObjectStores:n,foundObjectStores:t})),
E.return(a());case 6:throw new Kk(t,n);case 5:return E.return(k);case 2:y=wa(E);if(y instanceof DOMException?"VersionError"!==y.name:"DOMError"in self&&y instanceof DOMError?"VersionError"!==y.name:!(y instanceof Object&&"message"in y)||"An attempt was made to open a database using a lower version than the existing version."!==y.message){E.s(10);break}return v(E,c.j(c.name,void 0,Object.assign(Object.assign({},e),{upgrade:void 0})),11);case 11:u=E.i;D=u.h.version;if(void 0!==c.options.version&&D>
c.options.version+1)throw u.close(),c.l=!1,ul(c,D);return E.return(u);case 10:throw b(),y instanceof Error&&!K("ytidb_async_stack_killswitch")&&(y.stack=y.stack+"\n"+h.substring(h.indexOf("\n")+1)),Nk(y,c.name,"",null!==(g=c.options.version)&&void 0!==g?g:-1);}})}
function b(){c.h===d&&(c.h=void 0)}
var c=this;if(!this.l)throw ul(this);if(this.h)return this.h;var d,e={blocking:function(f){f.close()},
closed:b,yc:b,upgrade:this.options.upgrade};return this.h=d=a()};var wl=new tl("YtIdbMeta",{ya:{databases:{Wa:1}},upgrade:function(a,b){b(1)&&cl(a,"databases",{keyPath:"actualName"})}});
function xl(a,b){var c;return w(function(d){if(1==d.h)return v(d,vl(wl,b),2);c=d.i;return d.return(bl(c,["databases"],{O:!0,mode:"readwrite"},function(e){var f=e.objectStore("databases");return f.get(a.actualName).then(function(g){if(g?a.actualName!==g.actualName||a.publicName!==g.publicName||a.userIdentifier!==g.userIdentifier:1)return Zk(f.h.put(a,void 0)).then(function(){})})}))})}
function yl(a,b){var c;return w(function(d){if(1==d.h)return a?v(d,vl(wl,b),2):d.return();c=d.i;return d.return(c.delete("databases",a))})}
function zl(a,b){var c,d;return w(function(e){return 1==e.h?(c=[],v(e,vl(wl,b),2)):3!=e.h?(d=e.i,v(e,bl(d,["databases"],{O:!0,mode:"readonly"},function(f){c.length=0;return kl(f.objectStore("databases"),{},function(g){a(g.getValue())&&c.push(g.getValue());return g.continue()})}),3)):e.return(c)})}
function Al(a){return zl(function(b){return"LogsDatabaseV2"===b.publicName&&void 0!==b.userIdentifier},a)}
function Bl(a,b,c){return zl(function(d){return c?void 0!==d.userIdentifier&&!a.includes(d.userIdentifier)&&c.includes(d.publicName):void 0!==d.userIdentifier&&!a.includes(d.userIdentifier)},b)}
function Cl(a){var b,c;return w(function(d){if(1==d.h)return b=Ak("YtIdbMeta hasAnyMeta other"),v(d,zl(function(e){return void 0!==e.userIdentifier&&e.userIdentifier!==b},a),2);
c=d.i;return d.return(0<c.length)})}
;var Dl,El=new function(){}(new function(){});
function Fl(){var a,b,c;return w(function(d){switch(d.h){case 1:a=pk();if(null===a||void 0===a?0:a.hasSucceededOnce)return d.return(!0);var e;if(e=mk)e=/WebKit\/([0-9]+)/.exec(Ob()),e=!!(e&&600<=parseInt(e[1],10));e&&(e=/WebKit\/([0-9]+)/.exec(Ob()),e=!(e&&602<=parseInt(e[1],10)));if(e||jc)return d.return(!1);try{if(b=self,!(b.indexedDB&&b.IDBIndex&&b.IDBKeyRange&&b.IDBObjectStore))return d.return(!1)}catch(f){return d.return(!1)}if(!("IDBTransaction"in self&&"objectStoreNames"in IDBTransaction.prototype))return d.return(!1);
ua(d,2);c={actualName:"yt-idb-test-do-not-use",publicName:"yt-idb-test-do-not-use",userIdentifier:void 0};return v(d,xl(c,El),4);case 4:return v(d,yl("yt-idb-test-do-not-use",El),5);case 5:return d.return(!0);case 2:return wa(d),d.return(!1)}})}
function Gl(){if(void 0!==Dl)return Dl;sk=!0;return Dl=Fl().then(function(a){sk=!1;var b,c;null!==(b=ok())&&void 0!==b&&b.h&&(b=pk(),b={hasSucceededOnce:(null===b||void 0===b?void 0:b.hasSucceededOnce)||a},null===(c=ok())||void 0===c?void 0:c.set("LAST_RESULT_ENTRY_KEY",b,2592E3,!0));return a})}
function Hl(){return A("ytglobal.idbToken_")||void 0}
function Il(){var a=Hl();return a?Promise.resolve(a):Gl().then(function(b){(b=b?El:void 0)&&z("ytglobal.idbToken_",b,void 0);return b})}
;var Jl=0;function Kl(a,b){Jl||(Jl=ai.N(function(){var c,d,e,f,g;return w(function(h){switch(h.h){case 1:return v(h,Il(),2);case 2:c=h.i;if(!c)return h.return();d=!0;ua(h,3);return v(h,Bl(a,c,b),5);case 5:e=h.i;if(!e.length){d=!1;h.s(6);break}f=e[0];return v(h,rl(f.actualName),7);case 7:return v(h,yl(f.actualName,c),6);case 6:va(h,4);break;case 3:g=wa(h),xk(g),d=!1;case 4:ai.U(Jl),Jl=0,d&&Kl(a,b),h.h=0}})}))}
function Ll(){var a;return w(function(b){return 1==b.h?v(b,Il(),2):(a=b.i)?b.return(Cl(a)):b.return(!1)})}
new od;function Ml(a){if(!zk())throw a=new Jk("AUTH_INVALID",{dbName:a}),xk(a),a;var b=Ak();return{actualName:a+":"+b,publicName:a,userIdentifier:b}}
function Nl(a,b,c,d){var e,f,g,h,k,l;return w(function(n){switch(n.h){case 1:return f=null!==(e=Error().stack)&&void 0!==e?e:"",v(n,Il(),2);case 2:g=n.i;if(!g)throw h=Ok("openDbImpl",a,b),K("ytidb_async_stack_killswitch")||(h.stack=h.stack+"\n"+f.substring(f.indexOf("\n")+1)),xk(h),h;Bk(a);k=c?{actualName:a,publicName:a,userIdentifier:void 0}:Ml(a);ua(n,3);return v(n,xl(k,g),5);case 5:return v(n,ql(k.actualName,b,d),6);case 6:return n.return(n.i);case 3:return l=wa(n),ua(n,7),v(n,yl(k.actualName,
g),9);case 9:va(n,8);break;case 7:wa(n);case 8:throw l;}})}
function Ol(a,b,c){c=void 0===c?{}:c;return Nl(a,b,!1,c)}
function Pl(a,b,c){c=void 0===c?{}:c;return Nl(a,b,!0,c)}
function Ql(a,b){b=void 0===b?{}:b;var c,d;return w(function(e){if(1==e.h)return v(e,Il(),2);if(3!=e.h){c=e.i;if(!c)return e.return();Bk(a);d=Ml(a);return v(e,rl(d.actualName,b),3)}return v(e,yl(d.actualName,c),0)})}
function Rl(a,b,c){a=a.map(function(d){return w(function(e){return 1==e.h?v(e,rl(d.actualName,b),2):v(e,yl(d.actualName,c),0)})});
return Promise.all(a).then(function(){})}
function Sl(){var a=void 0===a?{}:a;var b,c;return w(function(d){if(1==d.h)return v(d,Il(),2);if(3!=d.h){b=d.i;if(!b)return d.return();Bk("LogsDatabaseV2");return v(d,Al(b),3)}c=d.i;return v(d,Rl(c,a,b),0)})}
function Tl(a,b){b=void 0===b?{}:b;var c;return w(function(d){if(1==d.h)return v(d,Il(),2);if(3!=d.h){c=d.i;if(!c)return d.return();Bk(a);return v(d,rl(a,b),3)}return v(d,yl(a,c),0)})}
;function Ul(a){var b,c,d,e,f,g,h,k;this.h=!1;this.potentialEsfErrorCounter=this.i=0;this.handleError=function(){};
this.ra=function(){};
this.now=Date.now;this.xa=!1;this.Jb=null!==(b=a.Jb)&&void 0!==b?b:100;this.Hb=null!==(c=a.Hb)&&void 0!==c?c:1;this.Fb=null!==(d=a.Fb)&&void 0!==d?d:2592E6;this.Db=null!==(e=a.Db)&&void 0!==e?e:12E4;this.Gb=null!==(f=a.Gb)&&void 0!==f?f:5E3;this.v=null!==(g=a.v)&&void 0!==g?g:void 0;this.La=!!a.La;this.Ka=null!==(h=a.Ka)&&void 0!==h?h:.1;this.Ra=null!==(k=a.Ra)&&void 0!==k?k:10;a.handleError&&(this.handleError=a.handleError);a.ra&&(this.ra=a.ra);a.xa&&(this.xa=a.xa);this.C=a.C;this.V=a.V;this.K=a.K;
this.J=a.J;this.ea=a.ea;this.ib=a.ib;this.hb=a.hb;this.v&&(!this.C||this.C("networkless_logging"))&&Vl(this)}
function Vl(a){a.v&&!a.xa&&(a.h=!0,a.La&&Math.random()<=a.Ka&&a.K.Pb(a.v),Wl(a),a.J.H()&&a.Ba(),a.C&&!a.C("use_new_nwl_initialization")&&(a.J.W(a.ib,a.Ba.bind(a)),a.J.W(a.hb,a.nb.bind(a))))}
m=Ul.prototype;m.writeThenSend=function(a,b){var c=this;b=void 0===b?{}:b;if(this.v&&this.h){var d={url:a,options:b,timestamp:this.now(),status:"NEW",sendCount:0};this.K.set(d,this.v).then(function(e){d.id=e;c.J.H()&&Xl(c,d)}).catch(function(e){Xl(c,d);
Yl(c,e)})}else this.ea(a,b)};
m.sendThenWrite=function(a,b,c){var d=this;b=void 0===b?{}:b;if(this.v&&this.h){var e={url:a,options:b,timestamp:this.now(),status:"NEW",sendCount:0};this.C&&this.C("nwl_skip_retry")&&(e.skipRetry=c);if(this.J.H()||this.C&&this.C("nwl_aggressive_send_then_write")&&!e.skipRetry){if(!e.skipRetry){var f=b.onError?b.onError:function(){};
b.onError=function(g,h){return w(function(k){if(1==k.h)return v(k,d.K.set(e,d.v).catch(function(l){Yl(d,l)}),2);
f(g,h);k.h=0})}}this.ea(a,b,e.skipRetry)}else this.K.set(e,this.v).catch(function(g){d.ea(a,b,e.skipRetry);
Yl(d,g)})}else this.ea(a,b,this.C&&this.C("nwl_skip_retry")&&c)};
m.sendAndWrite=function(a,b){var c=this;b=void 0===b?{}:b;if(this.v&&this.h){var d={url:a,options:b,timestamp:this.now(),status:"NEW",sendCount:0},e=!1,f=b.onSuccess?b.onSuccess:function(){};
d.options.onSuccess=function(g,h){void 0!==d.id?c.K.qa(d.id,c.v):e=!0;c.J.ca&&c.C&&c.C("vss_network_hint")&&c.J.ca(!0);f(g,h)};
this.ea(d.url,d.options);this.K.set(d,this.v).then(function(g){d.id=g;e&&c.K.qa(d.id,c.v)}).catch(function(g){Yl(c,g)})}else this.ea(a,b)};
m.Ba=function(){var a=this;if(!this.v)throw Ok("throttleSend");this.i||(this.i=this.V.N(function(){var b;return w(function(c){if(1==c.h)return v(c,a.K.wb("NEW",a.v),2);if(3!=c.h)return b=c.i,b?v(c,Xl(a,b),3):(a.nb(),c.return());a.i&&(a.i=0,a.Ba());c.h=0})},this.Jb))};
m.nb=function(){this.V.U(this.i);this.i=0};
function Xl(a,b){var c,d;return w(function(e){switch(e.h){case 1:if(!a.v)throw c=Ok("immediateSend"),c;if(void 0===b.id){e.s(2);break}return v(e,a.K.Zb(b.id,a.v),3);case 3:(d=e.i)?b=d:a.ra(Error("The request cannot be found in the database."));case 2:if(Zl(a,b,a.Fb)){e.s(4);break}a.ra(Error("Networkless Logging: Stored logs request expired age limit"));if(void 0===b.id){e.s(5);break}return v(e,a.K.qa(b.id,a.v),5);case 5:return e.return();case 4:b.skipRetry||(b=$l(a,b));if(!b){e.s(0);break}if(!b.skipRetry||
void 0===b.id){e.s(8);break}return v(e,a.K.qa(b.id,a.v),8);case 8:a.ea(b.url,b.options,!!b.skipRetry),e.h=0}})}
function $l(a,b){if(!a.v)throw Ok("updateRequestHandlers");var c=b.options.onError?b.options.onError:function(){};
b.options.onError=function(e,f){var g;return w(function(h){switch(h.h){case 1:g=am(f);if(!(a.C&&a.C("nwl_consider_error_code")&&g||a.C&&!a.C("nwl_consider_error_code")&&a.potentialEsfErrorCounter<=a.Ra)){h.s(2);break}if(!a.J.fa){h.s(3);break}return v(h,a.J.fa(),3);case 3:if(a.J.H()){h.s(2);break}c(e,f);if(!a.C||!a.C("nwl_consider_error_code")||void 0===(null===b||void 0===b?void 0:b.id)){h.s(6);break}return v(h,a.K.jb(b.id,a.v,!1),6);case 6:return h.return();case 2:if(a.C&&a.C("nwl_consider_error_code")&&
!g&&a.potentialEsfErrorCounter>a.Ra)return h.return();a.potentialEsfErrorCounter++;if(void 0===(null===b||void 0===b?void 0:b.id)){h.s(8);break}return b.sendCount<a.Hb?v(h,a.K.jb(b.id,a.v),12):v(h,a.K.qa(b.id,a.v),8);case 12:a.V.N(function(){a.J.H()&&a.Ba()},a.Gb);
case 8:c(e,f),h.h=0}})};
var d=b.options.onSuccess?b.options.onSuccess:function(){};
b.options.onSuccess=function(e,f){return w(function(g){if(1==g.h)return void 0===(null===b||void 0===b?void 0:b.id)?g.s(2):v(g,a.K.qa(b.id,a.v),2);a.J.ca&&a.C&&a.C("vss_network_hint")&&a.J.ca(!0);d(e,f);g.h=0})};
return b}
function Zl(a,b,c){b=b.timestamp;return a.now()-b>=c?!1:!0}
function Wl(a){if(!a.v)throw Ok("retryQueuedRequests");a.K.wb("QUEUED",a.v).then(function(b){b&&!Zl(a,b,a.Db)?a.V.N(function(){return w(function(c){if(1==c.h)return void 0===b.id?c.s(2):v(c,a.K.jb(b.id,a.v),2);Wl(a);c.h=0})}):a.J.H()&&a.Ba()})}
function Yl(a,b){a.Kb&&!a.J.H()?a.Kb(b):a.handleError(b)}
function am(a){var b;return(a=null===(b=null===a||void 0===a?void 0:a.error)||void 0===b?void 0:b.code)&&400<=a&&599>=a?!1:!0}
;function bm(a,b){this.version=a;this.args=b}
;function cm(a,b){this.topic=a;this.h=b}
cm.prototype.toString=function(){return this.topic};var dm=A("ytPubsub2Pubsub2Instance")||new J;J.prototype.subscribe=J.prototype.subscribe;J.prototype.unsubscribeByKey=J.prototype.va;J.prototype.publish=J.prototype.ka;J.prototype.clear=J.prototype.clear;z("ytPubsub2Pubsub2Instance",dm,void 0);var em=A("ytPubsub2Pubsub2SubscribedKeys")||{};z("ytPubsub2Pubsub2SubscribedKeys",em,void 0);var fm=A("ytPubsub2Pubsub2TopicToKeys")||{};z("ytPubsub2Pubsub2TopicToKeys",fm,void 0);var gm=A("ytPubsub2Pubsub2IsAsync")||{};z("ytPubsub2Pubsub2IsAsync",gm,void 0);
z("ytPubsub2Pubsub2SkipSubKey",null,void 0);function hm(a,b){var c=im();c&&c.publish.call(c,a.toString(),a,b)}
function jm(a){var b=km,c=im();if(!c)return 0;var d=c.subscribe(b.toString(),function(e,f){var g=A("ytPubsub2Pubsub2SkipSubKey");g&&g==d||(g=function(){if(em[d])try{if(f&&b instanceof cm&&b!=e)try{var h=b.h,k=f;if(!k.args||!k.version)throw Error("yt.pubsub2.Data.deserialize(): serializedData is incomplete.");try{if(!h.ja){var l=new h;h.ja=l.version}var n=h.ja}catch(L){}if(!n||k.version!=n)throw Error("yt.pubsub2.Data.deserialize(): serializedData version is incompatible.");try{n=Reflect;var t=n.construct;
var y=k.args,u=y.length;if(0<u){var D=Array(u);for(k=0;k<u;k++)D[k]=y[k];var E=D}else E=[];f=t.call(n,h,E)}catch(L){throw L.message="yt.pubsub2.Data.deserialize(): "+L.message,L;}}catch(L){throw L.message="yt.pubsub2.pubsub2 cross-binary conversion error for "+b.toString()+": "+L.message,L;}a.call(window,f)}catch(L){M(L)}},gm[b.toString()]?A("yt.scheduler.instance")?ai.N(g):Rh(g,0):g())});
em[d]=!0;fm[b.toString()]||(fm[b.toString()]=[]);fm[b.toString()].push(d);return d}
function lm(){var a=mm,b=jm(function(c){a.apply(void 0,arguments);nm(b)});
return b}
function nm(a){var b=im();b&&("number"===typeof a&&(a=[a]),fb(a,function(c){b.unsubscribeByKey(c);delete em[c]}))}
function im(){return A("ytPubsub2Pubsub2Instance")}
;function om(a,b){tl.call(this,a,b);this.options=b;Bk(a)}
r(om,tl);function pm(a,b){var c;return function(){c||(c=new om(a,b));return c}}
om.prototype.j=function(a,b,c){c=void 0===c?{}:c;return(this.options.kb?Pl:Ol)(a,b,Object.assign({},c))};
om.prototype.delete=function(a){a=void 0===a?{}:a;return(this.options.kb?Tl:Ql)(this.name,a)};
function qm(a,b){return pm(a,b)}
;var rm;
function sm(){if(rm)return rm();var a={};rm=qm("LogsDatabaseV2",{ya:(a.LogsRequestsStore={Wa:2},a),kb:!1,upgrade:function(b,c,d){c(2)&&cl(b,"LogsRequestsStore",{keyPath:"id",autoIncrement:!0});c(3);c(5)&&(d=d.objectStore("LogsRequestsStore"),d.h.indexNames.contains("newRequest")&&d.h.deleteIndex("newRequest"),d.h.createIndex("newRequestV2",["status","interface","timestamp"],{unique:!1}));c(7)&&b.h.objectStoreNames.contains("sapisid")&&b.h.deleteObjectStore("sapisid");c(9)&&b.h.objectStoreNames.contains("SWHealthLog")&&b.h.deleteObjectStore("SWHealthLog")},
version:9});return rm()}
;function tm(a){return vl(sm(),a)}
function um(a,b){var c,d,e,f;return w(function(g){if(1==g.h)return c={startTime:O(),transactionType:"YT_IDB_TRANSACTION_TYPE_WRITE"},v(g,tm(b),2);if(3!=g.h)return d=g.i,e=Object.assign(Object.assign({},a),{options:JSON.parse(JSON.stringify(a.options)),interface:B("INNERTUBE_CONTEXT_CLIENT_NAME",0)}),v(g,el(d,e),3);f=g.i;c.zc=O();vm(c);return g.return(f)})}
function wm(a,b){var c,d,e,f,g,h,k;return w(function(l){if(1==l.h)return c={startTime:O(),transactionType:"YT_IDB_TRANSACTION_TYPE_READ"},v(l,tm(b),2);if(3!=l.h)return d=l.i,e=B("INNERTUBE_CONTEXT_CLIENT_NAME",0),f=[a,e,0],g=[a,e,O()],h=IDBKeyRange.bound(f,g),k=void 0,v(l,bl(d,["LogsRequestsStore"],{mode:"readwrite",O:!0},function(n){return nl(n.objectStore("LogsRequestsStore").index("newRequestV2"),{query:h,direction:"prev"},function(t){t.getValue()&&(k=t.getValue(),"NEW"===a&&(k.status="QUEUED",
t.update(k)))})}),3);
c.zc=O();vm(c);return l.return(k)})}
function xm(a,b){var c;return w(function(d){if(1==d.h)return v(d,tm(b),2);c=d.i;return d.return(bl(c,["LogsRequestsStore"],{mode:"readwrite",O:!0},function(e){var f=e.objectStore("LogsRequestsStore");return f.get(a).then(function(g){if(g)return g.status="QUEUED",Zk(f.h.put(g,void 0)).then(function(){return g})})}))})}
function ym(a,b,c){c=void 0===c?!0:c;var d;return w(function(e){if(1==e.h)return v(e,tm(b),2);d=e.i;return e.return(bl(d,["LogsRequestsStore"],{mode:"readwrite",O:!0},function(f){var g=f.objectStore("LogsRequestsStore");return g.get(a).then(function(h){return h?(h.status="NEW",c&&(h.sendCount+=1),Zk(g.h.put(h,void 0)).then(function(){return h})):Sk.resolve(void 0)})}))})}
function zm(a,b){var c;return w(function(d){if(1==d.h)return v(d,tm(b),2);c=d.i;return d.return(c.delete("LogsRequestsStore",a))})}
function Am(a){var b,c;return w(function(d){if(1==d.h)return v(d,tm(a),2);b=d.i;c=O()-2592E6;return v(d,bl(b,["LogsRequestsStore"],{mode:"readwrite",O:!0},function(e){return kl(e.objectStore("LogsRequestsStore"),{},function(f){if(f.getValue().timestamp<=c)return f.delete().then(function(){return f.continue()})})}),0)})}
function Bm(){return w(function(a){return v(a,Sl(),0)})}
function vm(a){K("nwl_csi_killswitch")||.01>=Math.random()&&hm("nwl_transaction_latency_payload",a)}
;var Cm={},Dm=qm("ServiceWorkerLogsDatabase",{ya:(Cm.SWHealthLog={Wa:1},Cm),kb:!0,upgrade:function(a,b){b(1)&&cl(a,"SWHealthLog",{keyPath:"id",autoIncrement:!0}).h.createIndex("swHealthNewRequest",["interface","timestamp"],{unique:!1})},
version:1});function Em(a){return vl(Dm(),a)}
function Fm(a){var b,c;return w(function(d){if(1==d.h)return v(d,Em(a),2);b=d.i;c=O()-2592E6;return v(d,bl(b,["SWHealthLog"],{mode:"readwrite",O:!0},function(e){return kl(e.objectStore("SWHealthLog"),{},function(f){if(f.getValue().timestamp<=c)return f.delete().then(function(){return f.continue()})})}),0)})}
function Gm(a){var b;return w(function(c){if(1==c.h)return v(c,Em(a),2);b=c.i;return v(c,b.clear("SWHealthLog"),0)})}
;var Hm={},Im=0;
function Jm(a){var b=void 0===b?"":b;if(a)if(b)Gi(a,void 0,"POST",b,void 0);else if(B("USE_NET_AJAX_FOR_PING_TRANSPORT",!1))Gi(a,void 0,"GET","",void 0);else{b:{try{var c=new cb({url:a});if(c.j&&c.i||c.l){var d=Zb(a.match(Yb)[5]||null),e;if(!(e=!d||!d.endsWith("/aclk"))){var f=a.search(ec);d:{for(b=0;0<=(b=a.indexOf("ri",b))&&b<f;){var g=a.charCodeAt(b-1);if(38==g||63==g){var h=a.charCodeAt(b+2);if(!h||61==h||38==h||35==h){var k=b;break d}}b+=3}k=-1}if(0>k)var l=null;else{var n=a.indexOf("&",k);if(0>
n||n>f)n=f;k+=3;l=decodeURIComponent(a.substr(k,n-k).replace(/\+/g," "))}e="1"!==l}var t=!e;break b}}catch(u){}t=!1}if(t){b:{try{if(window.navigator&&window.navigator.sendBeacon&&window.navigator.sendBeacon(a,"")){var y=!0;break b}}catch(u){}y=!1}t=y?!0:!1}else t=!1;t||Km(a)}}
function Km(a){var b=new Image,c=""+Im++;Hm[c]=b;b.onload=b.onerror=function(){delete Hm[c]};
b.src=a}
;function Lm(){this.h=new Map;this.i=!1}
function Mm(){if(!Lm.h){var a=A("yt.networkRequestMonitor.instance")||new Lm;z("yt.networkRequestMonitor.instance",a,void 0);Lm.h=a}return Lm.h}
Lm.prototype.requestComplete=function(a,b){b&&(this.i=!0);a=this.removeParams(a);this.h.get(a)||this.h.set(a,b)};
Lm.prototype.isEndpointCFR=function(a){a=this.removeParams(a);return(a=this.h.get(a))?!1:!1===a&&this.i?!0:null};
Lm.prototype.removeParams=function(a){return a.split("?")[0]};
Lm.prototype.removeParams=Lm.prototype.removeParams;Lm.prototype.isEndpointCFR=Lm.prototype.isEndpointCFR;Lm.prototype.requestComplete=Lm.prototype.requestComplete;Lm.getInstance=Mm;var Nm;function Om(){Nm||(Nm=new gk("yt.offline"));return Nm}
function Pm(a){if(K("offline_error_handling")){var b=Om().get("errors",!0)||{};b[a.message]={name:a.name,stack:a.stack};a.level&&(b[a.message].level=a.level);Om().set("errors",b,2592E3,!0)}}
function Qm(){if(K("offline_error_handling")){var a=Om().get("errors",!0);if(a){for(var b in a)if(a[b]){var c=new N(b,"sent via offline_errors");c.name=a[b].name;c.stack=a[b].stack;c.level=a[b].level;M(c)}Om().set("errors",{},2592E3,!0)}}}
;var Rm=xh("network_polling_interval",3E4);function Q(){He.call(this);this.M=0;this.S=this.m=!1;this.l=this.ab();K("use_shared_nsm")?(Ke.h||(Ke.h=new Ke(ai)),this.j=Ke.h):(Sm(this),Tm(this))}
r(Q,He);function Um(){if(!Q.h){var a=A("yt.networkStatusManager.instance")||new Q;z("yt.networkStatusManager.instance",a,void 0);Q.h=a}return Q.h}
m=Q.prototype;m.H=function(){var a;return K("use_shared_nsm")&&this.j?null===(a=this.j)||void 0===a?void 0:a.H():this.l};
m.ca=function(a){var b;K("use_shared_nsm")&&this.j?null===(b=this.j)||void 0===b?void 0:b.j=a:a!==this.l&&(this.l=a)};
m.ac=function(a){!K("use_shared_nsm")&&(this.m=!0,void 0===a?0:a)&&(this.M||Vm(this))};
m.ab=function(){var a=window.navigator.onLine;return void 0===a?!0:a};
m.Sb=function(){this.S=!0};
m.W=function(a,b){return K("use_shared_nsm")&&this.j?this.j.W(a,b):He.prototype.W.call(this,a,b)};
function Tm(a){window.addEventListener("online",function(){return w(function(b){if(1==b.h)return v(b,a.fa(),2);a.S&&Qm();b.h=0})})}
function Sm(a){window.addEventListener("offline",function(){return w(function(b){return v(b,a.fa(),0)})})}
function Vm(a){a.M=Vh(function(){return w(function(b){if(1==b.h)return a.l?a.ab()||!a.m?b.s(3):v(b,a.fa(),3):v(b,a.fa(),3);Vm(a);b.h=0})},Rm)}
m.fa=function(a){var b=this;if(K("use_shared_nsm")&&this.j){var c=Le(this.j,a);c.then(function(d){K("use_cfr_monitor")&&Mm().requestComplete("generate_204",d)});
return c}return this.u?this.u:this.u=new Promise(function(d){var e,f,g;return w(function(h){switch(h.h){case 1:return e=window.AbortController?new window.AbortController:void 0,f=null===e||void 0===e?void 0:e.signal,g=!1,ua(h,2,3),e&&(b.A=ai.N(function(){e.abort()},a||2E4)),v(h,fetch("/generate_204",{method:"HEAD",
signal:f}),5);case 5:g=!0;case 3:xa(h);K("use_cfr_monitor")&&Mm().requestComplete("generate_204",g);b.u=void 0;b.A&&ai.U(b.A);g!==b.l&&(b.l=g,b.l&&b.m?Ie(b,"ytnetworkstatus-online"):b.m&&Ie(b,"ytnetworkstatus-offline"));d(g);ya(h);break;case 2:wa(h),g=!1,h.s(3)}})})};
Q.prototype.sendNetworkCheckRequest=Q.prototype.fa;Q.prototype.listen=Q.prototype.W;Q.prototype.enableErrorFlushing=Q.prototype.Sb;Q.prototype.getWindowStatus=Q.prototype.ab;Q.prototype.monitorNetworkStatusChange=Q.prototype.ac;Q.prototype.networkStatusHint=Q.prototype.ca;Q.prototype.isNetworkAvailable=Q.prototype.H;Q.getInstance=Um;function Wm(a){a=void 0===a?{}:a;He.call(this);var b=this;this.l=this.M=0;this.m="ytnetworkstatus-offline";this.u="ytnetworkstatus-online";K("use_shared_nsm")&&(this.m="networkstatus-offline",this.u="networkstatus-online");this.j=Um();var c=A("yt.networkStatusManager.instance.monitorNetworkStatusChange").bind(this.j);c&&c(a.tb);a.Oa&&!K("use_shared_nsm")&&(c=A("yt.networkStatusManager.instance.enableErrorFlushing").bind(this.j))&&c();if(c=A("yt.networkStatusManager.instance.listen").bind(this.j))a.Ta?
(this.Ta=a.Ta,c(this.u,function(){Xm(b,"publicytnetworkstatus-online");K("use_shared_nsm")&&a.Oa&&Qm()}),c(this.m,function(){Xm(b,"publicytnetworkstatus-offline")})):(c(this.u,function(){Ie(b,"publicytnetworkstatus-online");
K("use_shared_nsm")&&a.Oa&&Qm()}),c(this.m,function(){Ie(b,"publicytnetworkstatus-offline")}))}
r(Wm,He);Wm.prototype.H=function(){var a=A("yt.networkStatusManager.instance.isNetworkAvailable");return a?a.bind(this.j)():!0};
Wm.prototype.ca=function(a){var b=A("yt.networkStatusManager.instance.networkStatusHint").bind(this.j);b&&b(a)};
Wm.prototype.fa=function(a){var b=this,c;return w(function(d){c=A("yt.networkStatusManager.instance.sendNetworkCheckRequest").bind(b.j);return K("skip_network_check_if_cfr")&&Mm().isEndpointCFR("generate_204")?d.return(new Promise(function(e){var f;b.ca((null===(f=window.navigator)||void 0===f?void 0:f.onLine)||!0);e(b.H())})):c?d.return(c(a)):d.return(!0)})};
function Xm(a,b){a.Ta?a.l?(ai.U(a.M),a.M=ai.N(function(){a.A!==b&&(Ie(a,b),a.A=b,a.l=O())},a.Ta-(O()-a.l))):(Ie(a,b),a.A=b,a.l=O()):Ie(a,b)}
;var Ym;function Zm(){Ul.call(this,{K:{Pb:Am,qa:zm,wb:wm,Zb:xm,jb:ym,set:um},J:$m(),handleError:M,ra:Hh,ea:an,now:O,Kb:Pm,V:$h(),ib:"publicytnetworkstatus-online",hb:"publicytnetworkstatus-offline",La:!0,Ka:.1,Ra:xh("potential_esf_error_limit",10),C:K,xa:!(zk()&&bn())});this.j=new od;K("networkless_immediately_drop_all_requests")&&Bm();Tl("LogsDatabaseV2")}
r(Zm,Ul);function cn(){var a=A("yt.networklessRequestController.instance");a||(a=new Zm,z("yt.networklessRequestController.instance",a,void 0),K("networkless_logging")&&Il().then(function(b){a.v=b;Vl(a);a.j.resolve();a.La&&Math.random()<=a.Ka&&a.v&&Fm(a.v);K("networkless_immediately_drop_sw_health_store")&&dn(a)}));
return a}
Zm.prototype.writeThenSend=function(a,b){b||(b={});zk()||(this.h=!1);Ul.prototype.writeThenSend.call(this,a,b)};
Zm.prototype.sendThenWrite=function(a,b,c){b||(b={});zk()||(this.h=!1);Ul.prototype.sendThenWrite.call(this,a,b,c)};
Zm.prototype.sendAndWrite=function(a,b){b||(b={});zk()||(this.h=!1);Ul.prototype.sendAndWrite.call(this,a,b)};
Zm.prototype.awaitInitialization=function(){return this.j.promise};
function dn(a){var b;w(function(c){if(!a.v)throw b=Ok("clearSWHealthLogsDb"),b;return c.return(Gm(a.v).catch(function(d){a.handleError(d)}))})}
function an(a,b,c){K("use_cfr_monitor")&&en(a,b);var d;if(null===(d=b.postParams)||void 0===d?0:d.requestTimeMs)b.postParams.requestTimeMs=Math.round(O());c&&0===Object.keys(b).length?Jm(a):Di(a,b)}
function $m(){Ym||(Ym=new Wm({Oa:!0,tb:!0}));return Ym}
function en(a,b){var c=b.onError?b.onError:function(){};
b.onError=function(e,f){Mm().requestComplete(a,!1);c(e,f)};
var d=b.onSuccess?b.onSuccess:function(){};
b.onSuccess=function(e,f){Mm().requestComplete(a,!0);d(e,f)}}
function bn(){return K("embeds_web_nwl_disable_nocookie")?"www.youtube-nocookie.com"!==$b(document.location.toString()):!0}
;var fn=!1,gn=0,hn=0,jn,kn=x.ytNetworklessLoggingInitializationOptions||{isNwlInitialized:fn,potentialEsfErrorCounter:hn};z("ytNetworklessLoggingInitializationOptions",kn,void 0);
function ln(){var a;w(function(b){switch(b.h){case 1:return v(b,Il(),2);case 2:a=b.i;if(!a||!zk()&&!K("nwl_init_require_datasync_id_killswitch")||!bn()){b.s(0);break}fn=!0;kn.isNwlInitialized=fn;if(!K("use_new_nwl_initialization")){b.s(4);break}return v(b,cn().awaitInitialization(),5);case 5:return mn().W("publicytnetworkstatus-online",nn),mn().W("publicytnetworkstatus-offline",on),b.return();case 4:return v(b,Tl("LogsDatabaseV2"),6);case 6:if(!(.1>=Math.random())){b.s(7);break}return v(b,Am(a),8);
case 8:return v(b,Fm(a),7);case 7:pn();mn().H()&&nn();mn().W("publicytnetworkstatus-online",nn);mn().W("publicytnetworkstatus-offline",on);if(!K("networkless_immediately_drop_sw_health_store")){b.s(10);break}return v(b,qn(),10);case 10:if(K("networkless_immediately_drop_all_requests"))return v(b,Bm(),0);b.s(0)}})}
function rn(a,b){function c(d){var e=mn().H();if(!sn()||!d||e&&K("vss_networkless_bypass_write"))tn(a,b);else{var f={url:a,options:b,timestamp:O(),status:"NEW",sendCount:0};um(f,d).then(function(g){f.id=g;mn().H()&&un(f)}).catch(function(g){un(f);
mn().H()?M(g):Pm(g)})}}
b=void 0===b?{}:b;K("skip_is_supported_killswitch")?Il().then(function(d){c(d)}):c(Hl())}
function vn(a,b){function c(d){if(sn()&&d){var e={url:a,options:b,timestamp:O(),status:"NEW",sendCount:0},f=!1,g=b.onSuccess?b.onSuccess:function(){};
e.options.onSuccess=function(k,l){K("use_cfr_monitor")&&Mm().requestComplete(e.url,!0);void 0!==e.id?zm(e.id,d):f=!0;K("vss_network_hint")&&mn().ca(!0);g(k,l)};
if(K("use_cfr_monitor")){var h=b.onError?b.onError:function(){};
e.options.onError=function(k,l){Mm().requestComplete(e.url,!1);h(k,l)}}tn(e.url,e.options);
um(e,d).then(function(k){e.id=k;f&&zm(e.id,d)}).catch(function(k){mn().H()?M(k):Pm(k)})}else tn(a,b)}
b=void 0===b?{}:b;K("skip_is_supported_killswitch")?Il().then(function(d){c(d)}):c(Hl())}
function nn(){var a=Hl();if(!a)throw Ok("throttleSend");gn||(gn=ai.N(function(){var b;return w(function(c){if(1==c.h)return v(c,wm("NEW",a),2);if(3!=c.h)return b=c.i,b?v(c,un(b),3):(on(),c.return());gn&&(gn=0,nn());c.h=0})},100))}
function on(){ai.U(gn);gn=0}
function un(a){var b,c,d;return w(function(e){switch(e.h){case 1:b=Hl();if(!b)throw c=Ok("immediateSend"),c;if(void 0===a.id){e.s(2);break}return v(e,xm(a.id,b),3);case 3:(d=e.i)?a=d:Hh(Error("The request cannot be found in the database."));case 2:if(wn(a,2592E6)){e.s(4);break}Hh(Error("Networkless Logging: Stored logs request expired age limit"));if(void 0===a.id){e.s(5);break}return v(e,zm(a.id,b),5);case 5:return e.return();case 4:a.skipRetry||(a=xn(a));var f=a,g,h;if(null===(h=null===(g=null===
f||void 0===f?void 0:f.options)||void 0===g?void 0:g.postParams)||void 0===h?0:h.requestTimeMs)f.options.postParams.requestTimeMs=Math.round(O());a=f;if(!a){e.s(0);break}if(!a.skipRetry||void 0===a.id){e.s(8);break}return v(e,zm(a.id,b),8);case 8:tn(a.url,a.options,!!a.skipRetry),e.h=0}})}
function xn(a){var b=Hl();if(!b)throw Ok("updateRequestHandlers");var c=a.options.onError?a.options.onError:function(){};
a.options.onError=function(e,f){var g;return w(function(h){switch(h.h){case 1:K("use_cfr_monitor")&&Mm().requestComplete(a.url,!1);g=am(f);if(!(K("nwl_consider_error_code")&&g||!K("nwl_consider_error_code")&&yn()<=xh("potential_esf_error_limit",10))){h.s(2);break}if(K("skip_checking_network_on_cfr_failure")&&(!K("skip_checking_network_on_cfr_failure")||Mm().isEndpointCFR(a.url))){h.s(3);break}return v(h,mn().fa(),3);case 3:if(mn().H()){h.s(2);break}c(e,f);if(!K("nwl_consider_error_code")||void 0===
(null===a||void 0===a?void 0:a.id)){h.s(6);break}return v(h,ym(a.id,b,!1),6);case 6:return h.return();case 2:if(K("nwl_consider_error_code")&&!g&&yn()>xh("potential_esf_error_limit",10))return h.return();A("ytNetworklessLoggingInitializationOptions")&&kn.potentialEsfErrorCounter++;hn++;if(void 0===(null===a||void 0===a?void 0:a.id)){h.s(8);break}return 1>a.sendCount?v(h,ym(a.id,b),12):v(h,zm(a.id,b),8);case 12:ai.N(function(){mn().H()&&nn()},5E3);
case 8:c(e,f),h.h=0}})};
var d=a.options.onSuccess?a.options.onSuccess:function(){};
a.options.onSuccess=function(e,f){return w(function(g){if(1==g.h)return K("use_cfr_monitor")&&Mm().requestComplete(a.url,!0),void 0===(null===a||void 0===a?void 0:a.id)?g.s(2):v(g,zm(a.id,b),2);K("vss_network_hint")&&mn().ca(!0);d(e,f);g.h=0})};
return a}
function wn(a,b){a=a.timestamp;return O()-a>=b?!1:!0}
function pn(){var a=Hl();if(!a)throw Ok("retryQueuedRequests");wm("QUEUED",a).then(function(b){b&&!wn(b,12E4)?ai.N(function(){return w(function(c){if(1==c.h)return void 0===b.id?c.s(2):v(c,ym(b.id,a),2);pn();c.h=0})}):mn().H()&&nn()})}
function qn(){var a,b;return w(function(c){a=Hl();if(!a)throw b=Ok("clearSWHealthLogsDb"),b;return c.return(Gm(a).catch(function(d){M(d)}))})}
function mn(){if(K("use_new_nwl"))return $m();jn||(jn=new Wm({Oa:!0,tb:!0}));return jn}
function tn(a,b,c){c&&0===Object.keys(b).length?Jm(a):Di(a,b)}
function sn(){return A("ytNetworklessLoggingInitializationOptions")?kn.isNwlInitialized:fn}
function yn(){return A("ytNetworklessLoggingInitializationOptions")?kn.potentialEsfErrorCounter:hn}
;function zn(a){var b=this;this.config_=null;a?this.config_=a:Yj()&&(this.config_=rj());Vh(function(){lk(b)},5E3)}
zn.prototype.isReady=function(){!this.config_&&Yj()&&(this.config_=rj());return!!this.config_};
function vj(a,b,c,d){function e(D){D=void 0===D?!1:D;var E;if(d.retry&&"www.youtube-nocookie.com"!=h&&(D||K("skip_ls_gel_retry")||"application/json"!==g.headers["Content-Type"]||(E=jk(b,c,l,k)),E)){var L=g.onSuccess,P=g.onFetchSuccess;g.onSuccess=function(R,V){kk(E);L(R,V)};
c.onFetchSuccess=function(R,V){kk(E);P(R,V)}}try{D&&d.retry&&!d.Bb.bypassNetworkless?(g.method="POST",d.Bb.writeThenSend?K("use_new_nwl")?cn().writeThenSend(u,g):rn(u,g):K("use_new_nwl")?cn().sendAndWrite(u,g):vn(u,g)):(g.method="POST",g.postParams||(g.postParams={}),Di(u,g))}catch(R){if("InvalidAccessError"==R.name)E&&(kk(E),E=0),Hh(Error("An extension is blocking network request."));
else throw R;}E&&Vh(function(){lk(a)},5E3)}
!B("VISITOR_DATA")&&"visitor_id"!==b&&.01>Math.random()&&Hh(new N("Missing VISITOR_DATA when sending innertube request.",b,c,d));if(!a.isReady()){var f=new N("innertube xhrclient not ready",b,c,d);M(f);throw f;}var g={headers:d.headers||{},method:"POST",postParams:c,postBody:d.postBody,postBodyFormat:d.postBodyFormat||"JSON",onTimeout:function(){d.onTimeout()},
onFetchTimeout:d.onTimeout,onSuccess:function(D,E){if(d.onSuccess)d.onSuccess(E)},
onFetchSuccess:function(D){if(d.onSuccess)d.onSuccess(D)},
onError:function(D,E){if(d.onError)d.onError(E)},
onFetchError:function(D){if(d.onError)d.onError(D)},
timeout:d.timeout,withCredentials:!0};g.headers["Content-Type"]||(g.headers["Content-Type"]="application/json");var h="";(f=a.config_.Wb)&&(h=f);var k=a.config_.Yb||!1,l=ek(k,h,d);Object.assign(g.headers,l);(f=g.headers.Authorization)&&!h&&(g.headers["x-origin"]=window.location.origin);var n="/youtubei/"+a.config_.innertubeApiVersion+"/"+b,t={alt:"json"},y=a.config_.Xb&&f;y=y&&f.startsWith("Bearer");y||(t.key=a.config_.innertubeApiKey);var u=ri(""+h+n,t||{},!0);K("use_new_nwl")&&cn().h||!K("use_new_nwl")&&
sn()?Gl().then(function(D){e(D)}):e(!1)}
;function wk(a,b,c){c=void 0===c?{}:c;var d=zn;B("ytLoggingEventsDefaultDisabled",!1)&&zn==zn&&(d=null);Dj(a,b,d,c)}
;var An=[{gb:function(a){return"Cannot read property '"+a.key+"'"},
Qa:{Error:[{regexp:/(Permission denied) to access property "([^']+)"/,groups:["reason","key"]}],TypeError:[{regexp:/Cannot read property '([^']+)' of (null|undefined)/,groups:["key","value"]},{regexp:/\u65e0\u6cd5\u83b7\u53d6\u672a\u5b9a\u4e49\u6216 (null|undefined) \u5f15\u7528\u7684\u5c5e\u6027\u201c([^\u201d]+)\u201d/,groups:["value","key"]},{regexp:/\uc815\uc758\ub418\uc9c0 \uc54a\uc74c \ub610\ub294 (null|undefined) \ucc38\uc870\uc778 '([^']+)' \uc18d\uc131\uc744 \uac00\uc838\uc62c \uc218 \uc5c6\uc2b5\ub2c8\ub2e4./,
groups:["value","key"]},{regexp:/No se puede obtener la propiedad '([^']+)' de referencia nula o sin definir/,groups:["key"]},{regexp:/Unable to get property '([^']+)' of (undefined or null) reference/,groups:["key","value"]},{regexp:/(null) is not an object \(evaluating '(?:([^.]+)\.)?([^']+)'\)/,groups:["value","base","key"]}]}},{gb:function(a){return"Cannot call '"+a.key+"'"},
Qa:{TypeError:[{regexp:/(?:([^ ]+)?\.)?([^ ]+) is not a function/,groups:["base","key"]},{regexp:/([^ ]+) called on (null or undefined)/,groups:["key","value"]},{regexp:/Object (.*) has no method '([^ ]+)'/,groups:["base","key"]},{regexp:/Object doesn't support property or method '([^ ]+)'/,groups:["key"]},{regexp:/\u30aa\u30d6\u30b8\u30a7\u30af\u30c8\u306f '([^']+)' \u30d7\u30ed\u30d1\u30c6\u30a3\u307e\u305f\u306f\u30e1\u30bd\u30c3\u30c9\u3092\u30b5\u30dd\u30fc\u30c8\u3057\u3066\u3044\u307e\u305b\u3093/,
groups:["key"]},{regexp:/\uac1c\uccb4\uac00 '([^']+)' \uc18d\uc131\uc774\ub098 \uba54\uc11c\ub4dc\ub97c \uc9c0\uc6d0\ud558\uc9c0 \uc54a\uc2b5\ub2c8\ub2e4./,groups:["key"]}]}},{gb:function(a){return a.key+" is not defined"},
Qa:{ReferenceError:[{regexp:/(.*) is not defined/,groups:["key"]},{regexp:/Can't find variable: (.*)/,groups:["key"]}]}}];var Cn={ia:[],ha:[{la:Bn,weight:500}]};function Bn(a){if("JavaException"===a.name)return!0;a=a.stack;return a.includes("chrome://")||a.includes("chrome-extension://")||a.includes("moz-extension://")}
;function Dn(){this.ha=[];this.ia=[]}
var En;function Fn(){if(!En){var a=En=new Dn;a.ia.length=0;a.ha.length=0;Cn.ia&&a.ia.push.apply(a.ia,Cn.ia);Cn.ha&&a.ha.push.apply(a.ha,Cn.ha)}return En}
;var Gn=new J;function Hn(a){function b(){return a.charCodeAt(d++)}
var c=a.length,d=0;do{var e=In(b);if(Infinity===e)break;var f=e>>3;switch(e&7){case 0:e=In(b);if(2===f)return e;break;case 1:if(2===f)return;d+=8;break;case 2:e=In(b);if(2===f)return a.substr(d,e);d+=e;break;case 5:if(2===f)return;d+=4;break;default:return}}while(d<c)}
function In(a){var b=a(),c=b&127;if(128>b)return c;b=a();c|=(b&127)<<7;if(128>b)return c;b=a();c|=(b&127)<<14;if(128>b)return c;b=a();return 128>b?c|(b&127)<<21:Infinity}
;function Jn(a,b,c,d){if(a)if(Array.isArray(a)){var e=d;for(d=0;d<a.length&&!(a[d]&&(e+=Kn(d,a[d],b,c),500<e));d++);d=e}else if("object"===typeof a)for(e in a){if(a[e]){var f=a[e];var g=b;var h=c;g="string"!==typeof f||"clickTrackingParams"!==e&&"trackingParams"!==e?0:(f=Hn(atob(f.replace(/-/g,"+").replace(/_/g,"/"))))?Kn(e+".ve",f,g,h):0;d+=g;d+=Kn(e,a[e],b,c);if(500<d)break}}else c[b]=Ln(a),d+=c[b].length;else c[b]=Ln(a),d+=c[b].length;return d}
function Kn(a,b,c,d){c+="."+a;a=Ln(b);d[c]=a;return c.length+a.length}
function Ln(a){try{return("string"===typeof a?a:String(JSON.stringify(a))).substr(0,500)}catch(b){return"unable to serialize "+typeof a+" ("+b.message+")"}}
;var Mn=new Set,Nn=0,On=0,Pn=0,Qn=[],Rn=["PhantomJS","Googlebot","TO STOP THIS SECURITY SCAN go/scan"];function vk(a){Sn(a)}
function Tn(a){Sn(a,"WARNING")}
function Sn(a,b,c,d,e,f){f=void 0===f?{}:f;f.name=c||B("INNERTUBE_CONTEXT_CLIENT_NAME",1);f.version=d||B("INNERTUBE_CONTEXT_CLIENT_VERSION",void 0);c=f||{};b=void 0===b?"ERROR":b;b=void 0===b?"ERROR":b;if(a&&(a.hasOwnProperty("level")&&a.level&&(b=a.level),K("console_log_js_exceptions")&&(d=[],d.push("Name: "+a.name),d.push("Message: "+a.message),a.hasOwnProperty("params")&&d.push("Error Params: "+JSON.stringify(a.params)),a.hasOwnProperty("args")&&d.push("Error args: "+JSON.stringify(a.args)),d.push("File name: "+
a.fileName),d.push("Stacktrace: "+a.stack),window.console.log(d.join("\n"),a)),!(5<=Nn))){d=Qn;var g=fe(a);e=g.message||"Unknown Error";f=g.name||"UnknownError";var h=g.stack||a.i||"Not available";if(h.startsWith(f+": "+e)){var k=h.split("\n");k.shift();h=k.join("\n")}k=g.lineNumber||"Not available";g=g.fileName||"Not available";var l=0;if(a.hasOwnProperty("args")&&a.args&&a.args.length)for(var n=0;n<a.args.length&&!(l=Jn(a.args[n],"params."+n,c,l),500<=l);n++);else if(a.hasOwnProperty("params")&&
a.params){var t=a.params;if("object"===typeof a.params)for(n in t){if(t[n]){var y="params."+n,u=Ln(t[n]);c[y]=u;l+=y.length+u.length;if(500<l)break}}else c.params=Ln(t)}if(d.length)for(n=0;n<d.length&&!(l=Jn(d[n],"params.context."+n,c,l),500<=l);n++);navigator.vendor&&!c.hasOwnProperty("vendor")&&(c["device.vendor"]=navigator.vendor);n={message:e,name:f,lineNumber:k,fileName:g,stack:h,params:c,sampleWeight:1};c=Number(a.columnNumber);isNaN(c)||(n.lineNumber=n.lineNumber+":"+c);if("IGNORED"===a.level)a=
0;else a:{a=Fn();c=q(a.ia);for(d=c.next();!d.done;d=c.next())if(d=d.value,n.message&&n.message.match(d.Bo)){a=d.weight;break a}a=q(a.ha);for(c=a.next();!c.done;c=a.next())if(c=c.value,c.la(n)){a=c.weight;break a}a=1}n.sampleWeight=a;a=q(An);for(c=a.next();!c.done;c=a.next())if(c=c.value,c.Qa[n.name])for(e=q(c.Qa[n.name]),d=e.next();!d.done;d=e.next())if(f=d.value,d=n.message.match(f.regexp)){n.params["params.error.original"]=d[0];e=f.groups;f={};for(k=0;k<e.length;k++)f[e[k]]=d[k+1],n.params["params.error."+
e[k]]=d[k+1];n.message=c.gb(f);break}n.params||(n.params={});a=Fn();n.params["params.errorServiceSignature"]="msg="+a.ia.length+"&cb="+a.ha.length;n.params["params.serviceWorker"]="false";x.document&&x.document.querySelectorAll&&(n.params["params.fscripts"]=String(document.querySelectorAll("script:not([nonce])").length));Cb("sample").constructor!==Ab&&(n.params["params.fconst"]="true");window.yterr&&"function"===typeof window.yterr&&window.yterr(n);if(0!==n.sampleWeight&&!Mn.has(n.message)){"ERROR"===
b?(Gn.ka("handleError",n),K("record_app_crashed_web")&&0===Pn&&1===n.sampleWeight&&(Pn++,a={appCrashType:"APP_CRASH_TYPE_BREAKPAD"},K("report_client_error_with_app_crash_ks")||(a.systemHealth={crashData:{clientError:{logMessage:{message:n.message}}}}),wk("appCrashed",a)),On++):"WARNING"===b&&Gn.ka("handleWarning",n);if(K("kevlar_gel_error_routing")){a=b;b:{c=q(Rn);for(d=c.next();!d.done;d=c.next())if(nk(d.value.toLowerCase())){c=!0;break b}c=!1}if(c)c=void 0;else{d={stackTrace:n.stack};n.fileName&&
(d.filename=n.fileName);c=n.lineNumber&&n.lineNumber.split?n.lineNumber.split(":"):[];0!==c.length&&(1!==c.length||isNaN(Number(c[0]))?2!==c.length||isNaN(Number(c[0]))||isNaN(Number(c[1]))||(d.lineNumber=Number(c[0]),d.columnNumber=Number(c[1])):d.lineNumber=Number(c[0]));c={level:"ERROR_LEVEL_UNKNOWN",message:n.message,errorClassName:n.name,sampleWeight:n.sampleWeight};"ERROR"===a?c.level="ERROR_LEVEL_ERROR":"WARNING"===a&&(c.level="ERROR_LEVEL_WARNNING");d={isObfuscated:!0,browserStackInfo:d};
e={pageUrl:window.location.href,kvPairs:[]};B("FEXP_EXPERIMENTS")&&(e.experimentIds=B("FEXP_EXPERIMENTS"));f=B("LATEST_ECATCHER_SERVICE_TRACKING_PARAMS",void 0);k=oh.EXPERIMENT_FLAGS;if((!k||!k.web_disable_gel_stp_ecatcher_killswitch)&&f)for(g=q(Object.keys(f)),k=g.next();!k.done;k=g.next())k=k.value,e.kvPairs.push({key:k,value:String(f[k])});if(f=n.params)for(g=q(Object.keys(f)),k=g.next();!k.done;k=g.next())k=k.value,e.kvPairs.push({key:"client."+k,value:String(f[k])});f=vh("SERVER_NAME");k=vh("SERVER_VERSION");
f&&k&&(e.kvPairs.push({key:"server.name",value:f}),e.kvPairs.push({key:"server.version",value:k}));c={errorMetadata:e,stackTrace:d,logMessage:c}}c&&(wk("clientError",c),("ERROR"===a||K("errors_flush_gel_always_killswitch"))&&kj())}if(!K("suppress_error_204_logging")){a=n.params||{};b={urlParams:{a:"logerror",t:"jserror",type:n.name,msg:n.message.substr(0,250),line:n.lineNumber,level:b,"client.name":a.name},postParams:{url:B("PAGE_NAME",window.location.href),file:n.fileName},method:"POST"};a.version&&
(b["client.version"]=a.version);if(b.postParams){n.stack&&(b.postParams.stack=n.stack);c=q(Object.keys(a));for(d=c.next();!d.done;d=c.next())d=d.value,b.postParams["client."+d]=a[d];if(a=B("LATEST_ECATCHER_SERVICE_TRACKING_PARAMS",void 0))for(c=q(Object.keys(a)),d=c.next();!d.done;d=c.next())d=d.value,b.postParams[d]=a[d];a=B("SERVER_NAME",void 0);c=B("SERVER_VERSION",void 0);a&&c&&(b.postParams["server.name"]=a,b.postParams["server.version"]=c)}Di(B("ECATCHER_REPORT_HOST","")+"/error_204",b)}try{Mn.add(n.message)}catch(D){}Nn++}}}
function Un(a){var b=Fa.apply(1,arguments),c=a;c.args||(c.args=[]);c.args.push.apply(c.args,ia(b))}
;function Vn(){this.register=new Map}
function Wn(a){a=q(a.register.values());for(var b=a.next();!b.done;b=a.next())b.value.Eo("ABORTED")}
Vn.prototype.clear=function(){Wn(this);this.register.clear()};
var Xn=new Vn;var Yn=Date.now().toString();
function Zn(){a:{if(window.crypto&&window.crypto.getRandomValues)try{var a=Array(16),b=new Uint8Array(16);window.crypto.getRandomValues(b);for(var c=0;c<a.length;c++)a[c]=b[c];var d=a;break a}catch(e){}d=Array(16);for(a=0;16>a;a++){b=Date.now();for(c=0;c<b%23;c++)d[a]=Math.random();d[a]=Math.floor(256*Math.random())}if(Yn)for(a=1,b=0;b<Yn.length;b++)d[a%16]=d[a%16]^d[(a-1)%16]/4^Yn.charCodeAt(b),a++}a=[];for(b=0;b<d.length;b++)a.push("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_".charAt(d[b]&63));
return a.join("")}
;var $n,ao=x.ytLoggingDocDocumentNonce_;ao||(ao=Zn(),Xa("ytLoggingDocDocumentNonce_",ao));$n=ao;var bo={rg:0,md:1,ud:2,Tj:3,tg:4,Rn:5,Jk:6,jm:7,Ll:8,0:"DEFAULT",1:"CHAT",2:"CONVERSATIONS",3:"MINIPLAYER",4:"DIALOG",5:"VOZ",6:"MUSIC_WATCH_TABS",7:"SHARE",8:"PUSH_NOTIFICATIONS"};function co(a){this.h=a}
function eo(a){return new co({trackingParams:a})}
co.prototype.getAsJson=function(){var a={};void 0!==this.h.trackingParams?a.trackingParams=this.h.trackingParams:(a.veType=this.h.veType,void 0!==this.h.veCounter&&(a.veCounter=this.h.veCounter),void 0!==this.h.elementIndex&&(a.elementIndex=this.h.elementIndex));void 0!==this.h.dataElement&&(a.dataElement=this.h.dataElement.getAsJson());void 0!==this.h.youtubeData&&(a.youtubeData=this.h.youtubeData);return a};
co.prototype.getAsJspb=function(){var a=new Fg;void 0!==this.h.trackingParams?F(a,1,this.h.trackingParams):(void 0!==this.h.veType&&F(a,2,this.h.veType),void 0!==this.h.veCounter&&F(a,6,this.h.veCounter),void 0!==this.h.elementIndex&&F(a,3,this.h.elementIndex));if(void 0!==this.h.dataElement){var b=this.h.dataElement.getAsJspb();G(a,7,b)}void 0!==this.h.youtubeData&&G(a,8,this.h.jspbYoutubeData);return a};
co.prototype.toString=function(){return JSON.stringify(this.getAsJson())};
co.prototype.isClientVe=function(){return!this.h.trackingParams&&!!this.h.veType};function fo(a){a=void 0===a?0:a;return 0==a?"client-screen-nonce":"client-screen-nonce."+a}
function go(a){a=void 0===a?0:a;return 0==a?"ROOT_VE_TYPE":"ROOT_VE_TYPE."+a}
function ho(a){return B(go(void 0===a?0:a),void 0)}
z("yt_logging_screen.getRootVeType",ho,void 0);function io(a){return(a=ho(void 0===a?0:a))?new co({veType:a,youtubeData:void 0,jspbYoutubeData:void 0}):null}
function jo(){var a=B("csn-to-ctt-auth-info");a||(a={},uh("csn-to-ctt-auth-info",a));return a}
function ko(a){a=void 0===a?0:a;var b=B(fo(a));if(!b&&!B("USE_CSN_FALLBACK",!0))return null;b||!K("use_undefined_csn_any_layer")&&0!=a||(b="UNDEFINED_CSN");return b?b:null}
z("yt_logging_screen.getCurrentCsn",ko,void 0);function lo(a,b,c){var d=jo();(c=ko(c))&&delete d[c];b&&(d[a]=b)}
function mo(a){return jo()[a]}
z("yt_logging_screen.getCttAuthInfo",mo,void 0);function no(a,b,c,d){c=void 0===c?0:c;if(a!==B(fo(c))||b!==B(go(c)))lo(a,d,c),uh(fo(c),a),uh(go(c),b),b=function(){setTimeout(function(){if(a){var e={clientDocumentNonce:$n,clientScreenNonce:a};K("use_default_heartbeat_client")?wk("foregroundHeartbeatScreenAssociated",e):Dj("foregroundHeartbeatScreenAssociated",e,zn)}},0)},"requestAnimationFrame"in window?window.requestAnimationFrame(b):b()}
z("yt_logging_screen.setCurrentScreen",no,void 0);var oo=window.yt&&window.yt.msgs_||window.ytcfg&&window.ytcfg.msgs||{};z("yt.msgs_",oo,void 0);function po(a){jh(oo,arguments)}
;var qo={ld:3611,Ac:27686,Bc:85013,Cc:23462,Ec:42016,Fc:62407,Gc:26926,Dc:43781,Hc:51236,Ic:79148,Jc:50160,Kc:77504,Wc:87907,Xc:18630,Yc:54445,Zc:80935,bd:105675,cd:37521,dd:47786,ed:98349,fd:123695,gd:6827,hd:29434,jd:7282,kd:124448,od:32276,nd:76278,pd:93911,qd:106531,rd:27259,sd:27262,td:27263,vd:21759,wd:27107,xd:62936,yd:49568,zd:38408,Ad:80637,Bd:68727,Cd:68728,Dd:80353,Ed:80356,Fd:74610,Gd:45707,Hd:83962,Id:83970,Jd:46713,Kd:89711,Ld:74612,Md:93265,Nd:74611,Od:131380,Qd:128979,Rd:139311,Sd:128978,
Pd:131391,Td:105350,Vd:139312,Wd:134800,Ud:131392,Yd:113533,Zd:93252,ae:99357,ce:94521,de:114252,ee:113532,ge:94522,be:94583,he:88E3,ie:139580,je:93253,ke:93254,le:94387,me:94388,ne:93255,oe:97424,Xd:72502,pe:110111,qe:76019,se:117092,te:117093,re:89431,ue:110466,we:77240,xe:60508,ye:137401,ze:137402,Ae:137046,Be:73393,Ce:113534,De:92098,Ee:131381,Fe:84517,Ge:83759,He:80357,Ie:86113,Je:72598,Ke:72733,Le:107349,Me:124275,Ne:118203,Oe:133275,Pe:133274,Qe:133272,Re:133273,Se:133276,Te:144507,Ue:143247,
Ve:143248,We:143249,Xe:143250,Ye:143251,Ze:144401,bf:117431,af:133797,cf:128572,df:133405,ef:117429,ff:117430,gf:117432,hf:120080,jf:117259,kf:121692,lf:145656,mf:145655,nf:145653,pf:145654,qf:145657,rf:132972,sf:133051,tf:133658,uf:132971,vf:97615,xf:143359,wf:143356,zf:143361,yf:143358,Bf:143360,Af:143357,Cf:142303,Df:143353,Ef:143354,Ff:144479,Gf:143355,Hf:31402,Jf:133624,Kf:133623,Lf:133622,If:133621,Mf:84774,Nf:95117,Of:98930,Pf:98931,Qf:98932,Rf:43347,Sf:129889,Tf:45474,Uf:100352,Vf:84758,Wf:98443,
Xf:117985,Yf:74613,Zf:74614,ag:64502,cg:136032,dg:74615,eg:74616,fg:122224,gg:74617,hg:77820,ig:74618,jg:93278,kg:93274,lg:93275,mg:93276,ng:22110,og:29433,pg:133798,qg:132295,sg:120541,ug:82047,vg:113550,wg:75836,xg:75837,yg:42352,zg:84512,Ag:76065,Bg:75989,Cg:16623,Dg:32594,Eg:27240,Fg:32633,Gg:74858,Ig:3945,Hg:16989,Jg:45520,Kg:25488,Lg:25492,Mg:25494,Ng:55760,Og:14057,Pg:18451,Qg:57204,Rg:57203,Sg:17897,Tg:57205,Ug:18198,Vg:17898,Wg:17909,Xg:43980,Yg:46220,Zg:11721,ah:49954,bh:96369,dh:3854,eh:56251,
fh:25624,wh:16906,xh:99999,yh:68172,zh:27068,Ah:47973,Bh:72773,Ch:26970,Dh:26971,Eh:96805,Fh:17752,Gh:73233,Hh:109512,Ih:22256,Jh:14115,Kh:22696,Lh:89278,Mh:89277,Nh:109513,Oh:43278,Ph:43459,Qh:43464,Rh:89279,Sh:43717,Th:55764,Uh:22255,Vh:89281,Wh:40963,Xh:43277,Yh:43442,Zh:91824,ai:120137,bi:96367,ci:36850,di:72694,fi:37414,gi:36851,ii:124863,hi:121343,ji:73491,ki:54473,li:43375,mi:46674,ni:143815,oi:139095,ri:144402,si:32473,ti:72901,vi:72906,wi:50947,xi:50612,yi:50613,zi:50942,Ai:84938,Bi:84943,
Ci:84939,Di:84941,Ei:84944,Fi:84940,Gi:84942,Hi:35585,Ii:51926,Ji:79983,Ki:63238,Li:18921,Mi:63241,Ni:57893,Oi:41182,Pi:135732,Qi:33424,Ri:22207,Si:42993,Ti:36229,Ui:22206,Vi:22205,Wi:18993,Xi:19001,Yi:18990,Zi:18991,aj:18997,bj:18725,cj:19003,dj:36874,ej:44763,fj:33427,gj:67793,hj:22182,ij:37091,jj:34650,kj:50617,lj:47261,mj:22287,nj:25144,oj:97917,pj:62397,qj:125598,rj:137935,sj:36961,tj:108035,uj:27426,vj:27857,wj:27846,xj:27854,yj:69692,zj:61411,Aj:39299,Bj:38696,Cj:62520,Dj:36382,Ej:108701,Fj:50663,
Gj:36387,Hj:14908,Ij:37533,Jj:105443,Kj:61635,Lj:62274,Mj:133818,Nj:65702,Oj:65703,Pj:65701,Qj:76256,Rj:37671,Sj:49953,Uj:36216,Vj:28237,Wj:39553,Xj:29222,Yj:26107,Zj:38050,ak:26108,ck:120745,bk:26109,dk:26110,ek:66881,fk:28236,gk:14586,hk:57929,ik:74723,jk:44098,kk:44099,nk:23528,pk:61699,lk:134104,mk:134103,qk:59149,rk:101951,sk:97346,tk:118051,uk:95102,vk:64882,wk:119505,xk:63595,yk:63349,zk:95101,Ak:75240,Bk:27039,Ck:68823,Dk:21537,Ek:83464,Fk:75707,Gk:83113,Hk:101952,Ik:101953,Kk:79610,Lk:125755,
Mk:24402,Nk:24400,Ok:32925,Pk:57173,Qk:122502,Rk:145268,Sk:138480,Tk:64423,Uk:64424,Vk:33986,Wk:100828,Xk:129089,Yk:21409,dl:135155,fl:135156,il:135157,jl:135158,kl:135159,ll:135160,ml:135161,nl:135162,ol:135163,pl:135164,ql:135165,rl:135166,Zk:11070,al:11074,bl:17880,sl:14001,vl:30709,wl:30707,xl:30711,yl:30710,zl:30708,ul:26984,Al:63648,Bl:63649,Cl:51879,Dl:111059,El:5754,Fl:20445,Hl:130975,Gl:130976,Il:110386,Jl:113746,Kl:66557,Ml:17310,Nl:28631,Ol:21589,Pl:68012,Ql:60480,Rl:138664,Sl:141121,Tl:31571,
Ul:141978,Vl:76980,Wl:41577,Xl:45469,Yl:38669,Zl:13768,am:13777,bm:141842,cm:62985,dm:4724,em:59369,fm:43927,gm:43928,hm:12924,im:100355,lm:56219,mm:27669,nm:10337,km:47896,om:122629,qm:139723,pm:139722,rm:121258,sm:107598,tm:127991,um:96639,vm:107536,wm:130169,xm:96661,ym:145188,zm:96658,Am:116646,Bm:121122,Cm:96660,Dm:127738,Em:127083,Fm:104443,Gm:96659,Hm:106442,Im:134840,Jm:63667,Km:63668,Lm:63669,Mm:130686,Nm:78314,Om:55761,Pm:127098,Qm:134841,Rm:96368,Sm:67374,Tm:48992,Um:146176,Vm:49956,Wm:31961,
Xm:26388,Ym:23811,Zm:5E4,an:126250,bn:96370,cn:47355,dn:47356,en:37935,fn:45521,gn:21760,hn:83769,jn:49977,kn:49974,ln:93497,mn:93498,nn:34325,pn:140759,qn:115803,rn:123707,sn:100081,tn:35309,un:68314,vn:25602,wn:100339,xn:143516,yn:59018,zn:18248,An:50625,Bn:9729,Cn:37168,Dn:37169,En:21667,Fn:16749,Gn:18635,Hn:39305,In:18046,Jn:53969,Kn:8213,Ln:93926,Mn:102852,Nn:110099,On:22678,Pn:69076,Qn:137575,Sn:139224,Tn:100856,Un:17736,Vn:3832,Wn:55759,Xn:64031,eo:93044,fo:93045,ho:34388,jo:17657,ko:17655,
lo:39579,mo:39578,no:77448,oo:8196,po:11357,qo:69877,ro:8197,so:82039};function ro(){var a=ub(so),b;return Qf(new Jf(function(c,d){a.onSuccess=function(e){xi(e)?c(new to(e)):d(new uo("Request failed, status="+(e&&"status"in e?e.status:-1),"net.badstatus",e))};
a.onError=function(e){d(new uo("Unknown request error","net.unknown",e))};
a.onTimeout=function(e){d(new uo("Request timed out","net.timeout",e))};
b=Di("//googleads.g.doubleclick.net/pagead/id",a)}),function(c){c instanceof Rf&&b.abort();
return Of(c)})}
function uo(a,b,c){bb.call(this,a+", errorCode="+b);this.errorCode=b;this.xhr=c;this.name="PromiseAjaxError"}
r(uo,bb);function to(a){this.xhr=a}
;function vo(){this.i=0;this.h=null}
vo.prototype.then=function(a,b,c){return 1===this.i&&a?(a=a.call(c,this.h),If(a)?a:wo(a)):2===this.i&&b?(a=b.call(c,this.h),If(a)?a:xo(a)):this};
vo.prototype.getValue=function(){return this.h};
vo.prototype.$goog_Thenable=!0;function xo(a){var b=new vo;a=void 0===a?null:a;b.i=2;b.h=void 0===a?null:a;return b}
function wo(a){var b=new vo;a=void 0===a?null:a;b.i=1;b.h=void 0===a?null:a;return b}
;function yo(){if(Yd())return!0;var a=B("INNERTUBE_CLIENT_NAME");return!a||"WEB"!==a&&"MWEB"!==a||mk&&nk("applewebkit")&&!nk("version")&&(!nk("safari")||nk("gsa/"))||mc&&nk("version/")?!0:K("enable_web_eom_visitor_data")?B("EOM_VISITOR_DATA",void 0)?!1:!0:(a=Jj("CONSENT"))?a.startsWith("YES+"):!0}
;function zo(a){bb.call(this,a.message||a.description||a.name);this.isMissing=a instanceof Ao;this.isTimeout=a instanceof uo&&"net.timeout"==a.errorCode;this.isCanceled=a instanceof Rf}
r(zo,bb);zo.prototype.name="BiscottiError";function Ao(){bb.call(this,"Biscotti ID is missing from server")}
r(Ao,bb);Ao.prototype.name="BiscottiMissingError";var so={format:"RAW",method:"GET",timeout:5E3,withCredentials:!0},Bo=null;function hi(){if(K("disable_biscotti_fetch_entirely_for_all_web_clients"))return Of(Error("Biscotti id fetching has been disabled entirely."));if(!yo())return Of(Error("User has not consented - not fetching biscotti id."));if("1"==sb())return Of(Error("Biscotti ID is not available in private embed mode"));Bo||(Bo=Qf(ro().then(Co),function(a){return Do(2,a)}));
return Bo}
function Co(a){a=a.xhr.responseText;if(0!=a.lastIndexOf(")]}'",0))throw new Ao;a=JSON.parse(a.substr(4));if(1<(a.type||1))throw new Ao;a=a.id;ii(a);Bo=wo(a);Eo(18E5,2);return a}
function Do(a,b){b=new zo(b);ii("");Bo=xo(b);0<a&&Eo(12E4,a-1);throw b;}
function Eo(a,b){Rh(function(){Qf(ro().then(Co,function(c){return Do(b,c)}),La)},a)}
function Fo(){try{var a=A("yt.ads.biscotti.getId_");return a?a():hi()}catch(b){return Of(b)}}
;function Go(a){if("1"!=sb()){a&&gi();try{Fo().then(function(){},function(){}),Rh(Go,18E5)}catch(b){M(b)}}}
;function Ho(){this.xc=!0}
function Io(a){var b={},c=$d([]);c&&(b.Authorization=c,c=a=null===a||void 0===a?void 0:a.sessionIndex,void 0===c&&(c=Number(B("SESSION_INDEX",0)),c=isNaN(c)?0:c),b["X-Goog-AuthUser"]=c,"INNERTUBE_HOST_OVERRIDE"in oh||(b["X-Origin"]=window.location.origin),void 0===a&&"DELEGATED_SESSION_ID"in oh&&(b["X-Goog-PageId"]=B("DELEGATED_SESSION_ID")));return b}
;var Jo={identityType:"UNAUTHENTICATED_IDENTITY_TYPE_UNKNOWN"};var Ko=new Map([["dark","USER_INTERFACE_THEME_DARK"],["light","USER_INTERFACE_THEME_LIGHT"]]),Lo=["/fashion","/feed/fashion_destination","/channel/UCrpQ4p1Ql_hG8rKXIKM1MOQ"];function Mo(){var a=void 0===a?window.location.href:a;if(K("kevlar_disable_theme_param"))return null;var b=Zb(a.match(Yb)[5]||null);if(No(b))return"USER_INTERFACE_THEME_DARK";try{var c=qi(a).theme;return Ko.get(c)||null}catch(d){}return null}
function No(a){var b=Lo.map(function(c){return c.toLowerCase()});
return!K("disable_dark_fashion_destination_launch")&&b.some(function(c){return a.toLowerCase().startsWith(c)})?!0:!1}
;function Oo(){this.h={};if(this.i=Kj()){var a=Jj("CONSISTENCY");a&&Po(this,{encryptedTokenJarContents:a})}}
Oo.prototype.handleResponse=function(a,b){var c,d,e;b=(null===(d=null===(c=b.ba.context)||void 0===c?void 0:c.request)||void 0===d?void 0:d.consistencyTokenJars)||[];(a=null===(e=a.responseContext)||void 0===e?void 0:e.consistencyTokenJar)&&this.replace(b,a)};
Oo.prototype.replace=function(a,b){a=q(a);for(var c=a.next();!c.done;c=a.next())delete this.h[c.value.encryptedTokenJarContents];Po(this,b)};
function Po(a,b){if(b.encryptedTokenJarContents&&(a.h[b.encryptedTokenJarContents]=b,"string"===typeof b.expirationSeconds)){var c=Number(b.expirationSeconds);setTimeout(function(){delete a.h[b.encryptedTokenJarContents]},1E3*c);
a.i&&Ij("CONSISTENCY",b.encryptedTokenJarContents,c,void 0,!0)}}
;var Qo=window.location.hostname.split(".").slice(-2).join(".");function Ro(){var a=B("LOCATION_PLAYABILITY_TOKEN");"TVHTML5"===B("INNERTUBE_CLIENT_NAME")&&(this.h=So(this))&&(a=this.h.get("yt-location-playability-token"));a&&(this.locationPlayabilityToken=a,this.i=void 0)}
var To;Ro.getInstance=function(){To=A("yt.clientLocationService.instance");To||(To=new Ro,z("yt.clientLocationService.instance",To,void 0));return To};
Ro.prototype.setLocationOnInnerTubeContext=function(a){a.client||(a.client={});this.i?(a.client.locationInfo||(a.client.locationInfo={}),a.client.locationInfo.latitudeE7=1E7*this.i.coords.latitude,a.client.locationInfo.longitudeE7=1E7*this.i.coords.longitude,a.client.locationInfo.horizontalAccuracyMeters=this.i.coords.accuracy,a.client.locationInfo.forceLocationPlayabilityTokenRefresh=!0):this.locationPlayabilityToken&&(a.client.locationPlayabilityToken=this.locationPlayabilityToken)};
Ro.prototype.handleResponse=function(a){var b;a=null===(b=a.responseContext)||void 0===b?void 0:b.locationPlayabilityToken;void 0!==a&&(this.locationPlayabilityToken=a,this.i=void 0,"TVHTML5"===B("INNERTUBE_CLIENT_NAME")?(this.h=So(this))&&this.h.set("yt-location-playability-token",a,15552E3):Ij("YT_CL",JSON.stringify({loctok:a}),15552E3,Qo,!0))};
function So(a){return void 0===a.h?new gk("yt-client-location"):a.h}
Ro.prototype.getCurrentPositionFromGeolocation=function(){var a=this;if(!(navigator&&navigator.geolocation&&navigator.geolocation.getCurrentPosition)||!K("web_enable_browser_geolocation_api")&&!K("enable_handoff_location_2fa_on_mweb"))return Promise.reject(Error("Geolocation unsupported"));var b=!1,c=1E4;K("enable_handoff_location_2fa_on_mweb")&&(b=!0,c=15E3);return new Promise(function(d,e){navigator.geolocation.getCurrentPosition(function(f){a.i=f;d(f)},function(f){e(f)},{enableHighAccuracy:b,
maximumAge:0,timeout:c})})};
Ro.prototype.createUnpluggedLocationInfo=function(a){var b={};a=a.coords;if(null===a||void 0===a?0:a.latitude)b.latitudeE7=Math.floor(1E7*a.latitude);if(null===a||void 0===a?0:a.longitude)b.longitudeE7=Math.floor(1E7*a.longitude);if(null===a||void 0===a?0:a.accuracy)b.locationRadiusMeters=Math.round(a.accuracy);return b};function Uo(a,b){var c,d;if((null===(c=a.signalServiceEndpoint)||void 0===c?0:c.signal)&&b.Aa){var e=b.Aa[a.signalServiceEndpoint.signal];if(e)return e()}if((null===(d=a.continuationCommand)||void 0===d?0:d.request)&&b.Qb&&(e=b.Qb[a.continuationCommand.request]))return e();for(var f in a)if(b.pb[f]&&(a=b.pb[f]))return a()}
;var Vo=Symbol("injectionDeps");function Wo(){this.name="NETWORK_SLI_TOKEN"}
Wo.prototype.toString=function(){return"InjectionToken("+this.name+")"};function Xo(a){return function(){return new a}}
;var Yo={},Zo=(Yo.WEB_UNPLUGGED="^unplugged/",Yo.WEB_UNPLUGGED_ONBOARDING="^unplugged/",Yo.WEB_UNPLUGGED_OPS="^unplugged/",Yo.WEB_UNPLUGGED_PUBLIC="^unplugged/",Yo.WEB_CREATOR="^creator/",Yo.WEB_KIDS="^kids/",Yo.WEB_EXPERIMENTS="^experiments/",Yo.WEB_MUSIC="^music/",Yo.WEB_REMIX="^music/",Yo.WEB_MUSIC_EMBEDDED_PLAYER="^music/",Yo.WEB_MUSIC_EMBEDDED_PLAYER="^main_app/|^sfv/",Yo);
function $o(a){var b=void 0===b?"UNKNOWN_INTERFACE":b;if(1===a.length)return a[0];var c=Zo[b];if(c){var d=new RegExp(c),e=q(a);for(c=e.next();!c.done;c=e.next())if(c=c.value,d.exec(c))return c}var f=[];Object.entries(Zo).forEach(function(g){var h=q(g);g=h.next().value;h=h.next().value;b!==g&&f.push(h)});
d=new RegExp(f.join("|"));a.sort(function(g,h){return g.length-h.length});
e=q(a);for(c=e.next();!c.done;c=e.next())if(c=c.value,!d.exec(c))return c;return a[0]}
;function ap(a,b){return{method:void 0===b?"POST":b,mode:si(a)?"same-origin":"cors",credentials:si(a)?"same-origin":"include"}}
;function bp(){}
bp.prototype.o=function(a,b,c){b=void 0===b?{}:b;c=void 0===c?Jo:c;var d;var e=a.clickTrackingParams,f=this.l,g=!1;g=void 0===g?!1:g;f=void 0===f?!1:f;var h=B("INNERTUBE_CONTEXT");if(h){h=vb(h);K("web_no_tracking_params_in_shell_killswitch")||delete h.clickTracking;var k,l;h.client||(h.client={});var n=h.client;"MWEB"===n.clientName&&(n.clientFormFactor=B("IS_TABLET")?"LARGE_FORM_FACTOR":"SMALL_FORM_FACTOR");n.screenWidthPoints=window.innerWidth;n.screenHeightPoints=window.innerHeight;n.screenPixelDensity=
Math.round(window.devicePixelRatio||1);n.screenDensityFloat=window.devicePixelRatio||1;n.utcOffsetMinutes=-Math.floor((new Date).getTimezoneOffset());var t=void 0===t?!1:t;Mj.getInstance();var y="USER_INTERFACE_THEME_LIGHT";Pj(165)?y="USER_INTERFACE_THEME_DARK":Pj(174)?y="USER_INTERFACE_THEME_LIGHT":!K("kevlar_legacy_browsers")&&window.matchMedia&&window.matchMedia("(prefers-color-scheme)").matches&&window.matchMedia("(prefers-color-scheme: dark)").matches&&(y="USER_INTERFACE_THEME_DARK");t=t?y:Mo()||
y;n.userInterfaceTheme=t;if(!g){if(t=Wj())n.connectionType=t;K("web_log_effective_connection_type")&&(t=Xj())&&(h.client.effectiveConnectionType=t)}K("web_log_memory_total_kbytes")&&(null===(k=x.navigator)||void 0===k?0:k.deviceMemory)&&(k=null===(l=x.navigator)||void 0===l?void 0:l.deviceMemory,h.client.memoryTotalKbytes=""+1E6*k);l=qi(x.location.href);!K("web_populate_internal_geo_killswitch")&&l.internalcountrycode&&(n.internalGeo=l.internalcountrycode);"MWEB"===n.clientName||"WEB"===n.clientName?
(n.mainAppWebInfo={graftUrl:x.location.href},K("kevlar_woffle")&&Fj.h&&(n.mainAppWebInfo.pwaInstallabilityStatus=Fj.h.h?"PWA_INSTALLABILITY_STATUS_CAN_BE_INSTALLED":"PWA_INSTALLABILITY_STATUS_UNKNOWN"),n.mainAppWebInfo.webDisplayMode=Gj(),n.mainAppWebInfo.isWebNativeShareAvailable=navigator&&void 0!==navigator.share):"TVHTML5"===n.clientName&&(!K("web_lr_app_quality_killswitch")&&(l=B("LIVING_ROOM_APP_QUALITY"))&&(n.tvAppInfo=Object.assign(n.tvAppInfo||{},{appQuality:l})),l=B("LIVING_ROOM_CERTIFICATION_SCOPE"))&&
(n.tvAppInfo=Object.assign(n.tvAppInfo||{},{certificationScope:l}));if(!K("web_populate_time_zone_itc_killswitch")){b:{if("undefined"!==typeof Intl)try{var u=(new Intl.DateTimeFormat).resolvedOptions().timeZone;break b}catch(Ga){}u=void 0}u&&(n.timeZone=u)}(u=yh())?n.experimentsToken=u:delete n.experimentsToken;u=zh();Oo.h||(Oo.h=new Oo);n=Oo.h.h;l=[];k=0;for(var D in n)l[k++]=n[D];h.request=Object.assign(Object.assign({},h.request),{internalExperimentFlags:u,consistencyTokenJars:l});!K("web_prequest_context_killswitch")&&
(D=B("INNERTUBE_CONTEXT_PREQUEST_CONTEXT"))&&(h.request.externalPrequestContext=D);u=Mj.getInstance();D=Pj(58);u=u.get("gsml","");h.user=Object.assign({},h.user);D&&(h.user.enableSafetyMode=D);u&&(h.user.lockedSafetyMode=!0);K("warm_op_csn_cleanup")?f&&(g=ko())&&(h.clientScreenNonce=g):!g&&(g=ko())&&(h.clientScreenNonce=g);e&&(h.clickTracking={clickTrackingParams:e});if(e=A("yt.mdx.remote.remoteClient_"))h.remoteClient=e;K("web_enable_client_location_service")&&Ro.getInstance().setLocationOnInnerTubeContext(h);
try{var E=ti(void 0),L=E.bid;delete E.bid;h.adSignalsInfo={params:[],bid:L};for(var P=q(Object.entries(E)),R=P.next();!R.done;R=P.next()){var V=q(R.value),aa=V.next().value,Cd=V.next().value;E=aa;L=Cd;null===(d=h.adSignalsInfo.params)||void 0===d?void 0:d.push({key:E,value:""+L})}}catch(Ga){Sn(Ga)}d=h}else Sn(Error("Error: No InnerTubeContext shell provided in ytconfig.")),d={};d={context:d};if(P=this.h(a)){this.i(d,P,b);var ea,ja;b="/youtubei/v1/"+$o(this.j());(a=null===(ja=null===(ea=a.commandMetadata)||
void 0===ea?void 0:ea.webCommandMetadata)||void 0===ja?void 0:ja.apiUrl)&&(b=a);ea=b;(ja=B("INNERTUBE_HOST_OVERRIDE"))&&(ea=String(ja)+String(ac(ea)));ja={};ja.key=B("INNERTUBE_API_KEY");K("json_condensed_response")&&(ja.prettyPrint="false");ea=ri(ea,ja||{},!1);ea={input:ea,sa:ap(ea),ba:d,config:Object.assign({},void 0)};ea.config.Ha?ea.config.Ha.identity=c:ea.config.Ha={identity:c};return ea}Sn(new N("Error: Failed to create Request from Command.",a))};
fa.Object.defineProperties(bp.prototype,{l:{configurable:!0,enumerable:!0,get:function(){return!1}}});function cp(){}
r(cp,bp);cp.prototype.o=function(){return{input:"/getDatasyncIdsEndpoint",sa:ap("/getDatasyncIdsEndpoint","GET"),ba:{}}};
cp.prototype.j=function(){return[]};
cp.prototype.h=function(){};
cp.prototype.i=function(){};var dp={},ep=(dp.GET_DATASYNC_IDS=Xo(cp),dp);function fp(a){var b=Fa.apply(1,arguments);if(!gp(a)||b.some(function(e){return!gp(e)}))throw Error("Only objects may be merged.");
var c=a;b=q(b);for(var d=b.next();!d.done;d=b.next())hp(c,d.value);return c}
function hp(a,b){for(var c in b)if(gp(b[c])){if(c in a&&!gp(a[c]))throw Error("Cannot merge an object into a non-object.");c in a||(a[c]={});hp(a[c],b[c])}else if(ip(b[c])){if(c in a&&!ip(a[c]))throw Error("Cannot merge an array into a non-array.");c in a||(a[c]=[]);jp(a[c],b[c])}else a[c]=b[c];return a}
function jp(a,b){b=q(b);for(var c=b.next();!c.done;c=b.next())c=c.value,gp(c)?a.push(hp({},c)):ip(c)?a.push(jp([],c)):a.push(c);return a}
function gp(a){return"object"===typeof a&&!Array.isArray(a)}
function ip(a){return"object"===typeof a&&Array.isArray(a)}
;function kp(a,b){bm.call(this,1,arguments);this.timer=b}
r(kp,bm);var lp=new cm("aft-recorded",kp);var mp=window;function np(){this.timing={};this.clearResourceTimings=function(){};
this.webkitClearResourceTimings=function(){};
this.mozClearResourceTimings=function(){};
this.msClearResourceTimings=function(){};
this.oClearResourceTimings=function(){}}
var S=mp.performance||mp.mozPerformance||mp.msPerformance||mp.webkitPerformance||new np;var op=!1,pp={'script[name="scheduler/scheduler"]':"sj",'script[name="player/base"]':"pj",'link[rel="stylesheet"][name="www-player"]':"pc",'link[rel="stylesheet"][name="player/www-player"]':"pc",'script[name="desktop_polymer/desktop_polymer"]':"dpj",'link[rel="import"][name="desktop_polymer"]':"dph",'script[name="mobile-c3"]':"mcj",'link[rel="stylesheet"][name="mobile-c3"]':"mcc",'script[name="player-plasma-ias-phone/base"]':"mcppj",'script[name="player-plasma-ias-tablet/base"]':"mcptj",'link[rel="stylesheet"][name="mobile-polymer-player-ias"]':"mcpc",
'link[rel="stylesheet"][name="mobile-polymer-player-svg-ias"]':"mcpsc",'script[name="mobile_blazer_core_mod"]':"mbcj",'link[rel="stylesheet"][name="mobile_blazer_css"]':"mbc",'script[name="mobile_blazer_logged_in_users_mod"]':"mbliuj",'script[name="mobile_blazer_logged_out_users_mod"]':"mblouj",'script[name="mobile_blazer_noncore_mod"]':"mbnj","#player_css":"mbpc",'script[name="mobile_blazer_desktopplayer_mod"]':"mbpj",'link[rel="stylesheet"][name="mobile_blazer_tablet_css"]':"mbtc",'script[name="mobile_blazer_watch_mod"]':"mbwj"};
Va(S.clearResourceTimings||S.webkitClearResourceTimings||S.mozClearResourceTimings||S.msClearResourceTimings||S.oClearResourceTimings||La,S);function qp(a){var b=rp(a);if(b.aft)return b.aft;a=B((a||"")+"TIMING_AFT_KEYS",["ol"]);for(var c=a.length,d=0;d<c;d++){var e=b[a[d]];if(e)return e}return NaN}
function sp(){var a;if(K("csi_use_performance_navigation_timing")){var b,c,d,e=null===(d=null===(c=null===(b=null===(a=null===S||void 0===S?void 0:S.getEntriesByType)||void 0===a?void 0:a.call(S,"navigation"))||void 0===b?void 0:b[0])||void 0===c?void 0:c.toJSON)||void 0===d?void 0:d.call(c);e?(e.requestStart=tp(e.requestStart),e.responseEnd=tp(e.responseEnd),e.redirectStart=tp(e.redirectStart),e.redirectEnd=tp(e.redirectEnd),e.domainLookupEnd=tp(e.domainLookupEnd),e.connectStart=tp(e.connectStart),
e.connectEnd=tp(e.connectEnd),e.responseStart=tp(e.responseStart),e.secureConnectionStart=tp(e.secureConnectionStart),e.domainLookupStart=tp(e.domainLookupStart),e.isPerformanceNavigationTiming=!0,a=e):a=S.timing}else a=S.timing;return a}
function up(){return K("csi_use_time_origin")&&S.timeOrigin?Math.floor(S.timeOrigin):S.timing.navigationStart}
function tp(a){return Math.round(up()+a)}
function vp(a){var b;(b=A("ytcsi."+(a||"")+"data_"))||(b={tick:{},info:{}},Xa("ytcsi."+(a||"")+"data_",b));return b}
function wp(a){a=vp(a);a.info||(a.info={});return a.info}
function rp(a){a=vp(a);a.tick||(a.tick={});return a.tick}
function xp(a){a=vp(a);if(a.gel){var b=a.gel;b.gelInfos||(b.gelInfos={});b.gelTicks||(b.gelTicks={})}else a.gel={gelTicks:{},gelInfos:{}};return a.gel}
function yp(a){a=xp(a);a.gelInfos||(a.gelInfos={});return a.gelInfos}
function zp(a){var b=vp(a).nonce;b||(b=Zn(),vp(a).nonce=b);return b}
function Ap(a){var b=rp(a||""),c=qp(a);c&&!op&&(hm(lp,new kp(Math.round(c-b._start),a)),op=!0)}
function Bp(a,b){for(var c=q(Object.keys(b)),d=c.next();!d.done;d=c.next())if(d=d.value,!Object.keys(a).includes(d)||"object"===typeof b[d]&&!Bp(a[d],b[d]))return!1;return!0}
;function Cp(){if(S.getEntriesByType){var a=S.getEntriesByType("paint");if(a=jb(a,function(b){return"first-paint"===b.name}))return tp(a.startTime)}a=S.timing;
return a.cc?Math.max(0,a.cc):0}
;function Dp(){var a=A("ytcsi.debug");a||(a=[],z("ytcsi.debug",a,void 0),z("ytcsi.reference",{},void 0));return a}
function Ep(a){a=a||"";var b=A("ytcsi.reference");b||(Dp(),b=A("ytcsi.reference"));if(b[a])return b[a];var c=Dp(),d={timerName:a,info:{},tick:{},span:{},jspbInfo:[]};c.push(d);return b[a]=d}
;var T={},Fp=(T.auto_search="LATENCY_ACTION_AUTO_SEARCH",T.ad_to_ad="LATENCY_ACTION_AD_TO_AD",T.ad_to_video="LATENCY_ACTION_AD_TO_VIDEO",T["analytics.explore"]="LATENCY_ACTION_CREATOR_ANALYTICS_EXPLORE",T.app_startup="LATENCY_ACTION_APP_STARTUP",T["artist.analytics"]="LATENCY_ACTION_CREATOR_ARTIST_ANALYTICS",T["artist.events"]="LATENCY_ACTION_CREATOR_ARTIST_CONCERTS",T["artist.presskit"]="LATENCY_ACTION_CREATOR_ARTIST_PROFILE",T.browse="LATENCY_ACTION_BROWSE",T.cast_splash="LATENCY_ACTION_CAST_SPLASH",
T.channels="LATENCY_ACTION_CHANNELS",T.creator_channel_dashboard="LATENCY_ACTION_CREATOR_CHANNEL_DASHBOARD",T["channel.analytics"]="LATENCY_ACTION_CREATOR_CHANNEL_ANALYTICS",T["channel.comments"]="LATENCY_ACTION_CREATOR_CHANNEL_COMMENTS",T["channel.content"]="LATENCY_ACTION_CREATOR_POST_LIST",T["channel.copyright"]="LATENCY_ACTION_CREATOR_CHANNEL_COPYRIGHT",T["channel.editing"]="LATENCY_ACTION_CREATOR_CHANNEL_EDITING",T["channel.monetization"]="LATENCY_ACTION_CREATOR_CHANNEL_MONETIZATION",T["channel.music"]=
"LATENCY_ACTION_CREATOR_CHANNEL_MUSIC",T["channel.playlists"]="LATENCY_ACTION_CREATOR_CHANNEL_PLAYLISTS",T["channel.translations"]="LATENCY_ACTION_CREATOR_CHANNEL_TRANSLATIONS",T["channel.videos"]="LATENCY_ACTION_CREATOR_CHANNEL_VIDEOS",T["channel.live_streaming"]="LATENCY_ACTION_CREATOR_LIVE_STREAMING",T.chips="LATENCY_ACTION_CHIPS",T["dialog.copyright_strikes"]="LATENCY_ACTION_CREATOR_DIALOG_COPYRIGHT_STRIKES",T["dialog.uploads"]="LATENCY_ACTION_CREATOR_DIALOG_UPLOADS",T.direct_playback="LATENCY_ACTION_DIRECT_PLAYBACK",
T.embed="LATENCY_ACTION_EMBED",T.entity_key_serialization_perf="LATENCY_ACTION_ENTITY_KEY_SERIALIZATION_PERF",T.entity_key_deserialization_perf="LATENCY_ACTION_ENTITY_KEY_DESERIALIZATION_PERF",T.explore="LATENCY_ACTION_EXPLORE",T.home="LATENCY_ACTION_HOME",T.library="LATENCY_ACTION_LIBRARY",T.live="LATENCY_ACTION_LIVE",T.live_pagination="LATENCY_ACTION_LIVE_PAGINATION",T.onboarding="LATENCY_ACTION_ONBOARDING",T.parent_profile_settings="LATENCY_ACTION_KIDS_PARENT_PROFILE_SETTINGS",T.parent_tools_collection=
"LATENCY_ACTION_PARENT_TOOLS_COLLECTION",T.parent_tools_dashboard="LATENCY_ACTION_PARENT_TOOLS_DASHBOARD",T.player_att="LATENCY_ACTION_PLAYER_ATTESTATION",T["post.comments"]="LATENCY_ACTION_CREATOR_POST_COMMENTS",T["post.edit"]="LATENCY_ACTION_CREATOR_POST_EDIT",T.prebuffer="LATENCY_ACTION_PREBUFFER",T.prefetch="LATENCY_ACTION_PREFETCH",T.profile_settings="LATENCY_ACTION_KIDS_PROFILE_SETTINGS",T.profile_switcher="LATENCY_ACTION_LOGIN",T.reel_watch="LATENCY_ACTION_REEL_WATCH",T.results="LATENCY_ACTION_RESULTS",
T.search_ui="LATENCY_ACTION_SEARCH_UI",T.search_suggest="LATENCY_ACTION_SUGGEST",T.search_zero_state="LATENCY_ACTION_SEARCH_ZERO_STATE",T.secret_code="LATENCY_ACTION_KIDS_SECRET_CODE",T.seek="LATENCY_ACTION_PLAYER_SEEK",T.settings="LATENCY_ACTION_SETTINGS",T.tenx="LATENCY_ACTION_TENX",T.video_to_ad="LATENCY_ACTION_VIDEO_TO_AD",T.watch="LATENCY_ACTION_WATCH",T.watch_it_again="LATENCY_ACTION_KIDS_WATCH_IT_AGAIN",T["watch,watch7"]="LATENCY_ACTION_WATCH",T["watch,watch7_html5"]="LATENCY_ACTION_WATCH",
T["watch,watch7ad"]="LATENCY_ACTION_WATCH",T["watch,watch7ad_html5"]="LATENCY_ACTION_WATCH",T.wn_comments="LATENCY_ACTION_LOAD_COMMENTS",T.ww_rqs="LATENCY_ACTION_WHO_IS_WATCHING",T["video.analytics"]="LATENCY_ACTION_CREATOR_VIDEO_ANALYTICS",T["video.comments"]="LATENCY_ACTION_CREATOR_VIDEO_COMMENTS",T["video.edit"]="LATENCY_ACTION_CREATOR_VIDEO_EDIT",T["video.editor"]="LATENCY_ACTION_CREATOR_VIDEO_VIDEO_EDITOR",T["video.editor_async"]="LATENCY_ACTION_CREATOR_VIDEO_VIDEO_EDITOR_ASYNC",T["video.live_settings"]=
"LATENCY_ACTION_CREATOR_VIDEO_LIVE_SETTINGS",T["video.live_streaming"]="LATENCY_ACTION_CREATOR_VIDEO_LIVE_STREAMING",T["video.monetization"]="LATENCY_ACTION_CREATOR_VIDEO_MONETIZATION",T["video.translations"]="LATENCY_ACTION_CREATOR_VIDEO_TRANSLATIONS",T.voice_assistant="LATENCY_ACTION_VOICE_ASSISTANT",T.cast_load_by_entity_to_watch="LATENCY_ACTION_CAST_LOAD_BY_ENTITY_TO_WATCH",T.networkless_performance="LATENCY_ACTION_NETWORKLESS_PERFORMANCE",T),U={},Gp=(U.ad_allowed="adTypesAllowed",U.yt_abt="adBreakType",
U.ad_cpn="adClientPlaybackNonce",U.ad_docid="adVideoId",U.yt_ad_an="adNetworks",U.ad_at="adType",U.aida="appInstallDataAgeMs",U.browse_id="browseId",U.p="httpProtocol",U.t="transportProtocol",U.cs="commandSource",U.cpn="clientPlaybackNonce",U.ccs="creatorInfo.creatorCanaryState",U.ctop="creatorInfo.topEntityType",U.csn="clientScreenNonce",U.docid="videoId",U.GetHome_rid="requestIds",U.GetSearch_rid="requestIds",U.GetPlayer_rid="requestIds",U.GetWatchNext_rid="requestIds",U.GetBrowse_rid="requestIds",
U.GetLibrary_rid="requestIds",U.is_continuation="isContinuation",U.is_nav="isNavigation",U.b_p="kabukiInfo.browseParams",U.is_prefetch="kabukiInfo.isPrefetch",U.is_secondary_nav="kabukiInfo.isSecondaryNav",U.nav_type="kabukiInfo.navigationType",U.prev_browse_id="kabukiInfo.prevBrowseId",U.query_source="kabukiInfo.querySource",U.voz_type="kabukiInfo.vozType",U.yt_lt="loadType",U.mver="creatorInfo.measurementVersion",U.yt_ad="isMonetized",U.nr="webInfo.navigationReason",U.nrsu="navigationRequestedSameUrl",
U.pnt="performanceNavigationTiming",U.prt="playbackRequiresTap",U.plt="playerInfo.playbackType",U.pis="playerInfo.playerInitializedState",U.paused="playerInfo.isPausedOnLoad",U.yt_pt="playerType",U.fmt="playerInfo.itag",U.yt_pl="watchInfo.isPlaylist",U.yt_pre="playerInfo.preloadType",U.yt_ad_pr="prerollAllowed",U.pa="previousAction",U.yt_red="isRedSubscriber",U.rce="mwebInfo.responseContentEncoding",U.rc="resourceInfo.resourceCache",U.scrh="screenHeight",U.scrw="screenWidth",U.st="serverTimeMs",U.ssdm=
"shellStartupDurationMs",U.br_trs="tvInfo.bedrockTriggerState",U.kebqat="kabukiInfo.earlyBrowseRequestInfo.abandonmentType",U.kebqa="kabukiInfo.earlyBrowseRequestInfo.adopted",U.label="tvInfo.label",U.is_mdx="tvInfo.isMdx",U.preloaded="tvInfo.isPreloaded",U.aac_type="tvInfo.authAccessCredentialType",U.upg_player_vis="playerInfo.visibilityState",U.query="unpluggedInfo.query",U.upg_chip_ids_string="unpluggedInfo.upgChipIdsString",U.yt_vst="videoStreamType",U.vph="viewportHeight",U.vpw="viewportWidth",
U.yt_vis="isVisible",U.rcl="mwebInfo.responseContentLength",U.GetSettings_rid="requestIds",U.GetTrending_rid="requestIds",U.GetMusicSearchSuggestions_rid="requestIds",U.REQUEST_ID="requestIds",U),Hp="isContinuation isNavigation kabukiInfo.earlyBrowseRequestInfo.adopted kabukiInfo.isPrefetch kabukiInfo.isSecondaryNav isMonetized navigationRequestedSameUrl performanceNavigationTiming playerInfo.isPausedOnLoad prerollAllowed isRedSubscriber tvInfo.isMdx tvInfo.isPreloaded isVisible watchInfo.isPlaylist playbackRequiresTap".split(" "),
Ip={},Jp=(Ip.ccs="CANARY_STATE_",Ip.mver="MEASUREMENT_VERSION_",Ip.pis="PLAYER_INITIALIZED_STATE_",Ip.yt_pt="LATENCY_PLAYER_",Ip.pa="LATENCY_ACTION_",Ip.ctop="TOP_ENTITY_TYPE_",Ip.yt_vst="VIDEO_STREAM_TYPE_",Ip),Kp="all_vc ap aq c cbr cbrand cbrver cmodel cos cosver cplatform ctheme cver ei l_an l_mm plid srt yt_fss yt_li vpst vpni2 vpil2 icrc icrt pa GetAccountOverview_rid GetHistory_rid cmt d_vpct d_vpnfi d_vpni nsru pc pfa pfeh pftr pnc prerender psc rc start tcrt tcrc ssr vpr vps yt_abt yt_fn yt_fs yt_pft yt_pre yt_pt yt_pvis ytu_pvis yt_ref yt_sts tds".split(" ");
function Lp(a,b,c){c=xp(c);if(c.gelInfos)c.gelInfos[a]=!0;else{var d={};c.gelInfos=(d[a]=!0,d)}if(a.match("_rid")){var e=a.split("_rid")[0];a="REQUEST_ID"}if(a in Gp){c=Gp[a];0<=eb(Hp,c)&&(b=!!b);a in Jp&&"string"===typeof b&&(b=Jp[a]+b.toUpperCase());a=b;b=c.split(".");for(var f=d={},g=0;g<b.length-1;g++){var h=b[g];f[h]={};f=f[h]}f[b[b.length-1]]="requestIds"===c?[{id:a,endpoint:e}]:a;return fp({},d)}0<=eb(Kp,a)||Tn(new N("Unknown label logged with GEL CSI",a))}
;var W={LATENCY_ACTION_KIDS_PROFILE_SWITCHER:90,LATENCY_ACTION_OFFLINE_THUMBNAIL_TRANSFER:100,LATENCY_ACTION_CREATOR_VIDEO_VIDEO_EDITOR_ASYNC:46,LATENCY_ACTION_CREATOR_VIDEO_VIDEO_EDITOR:37,LATENCY_ACTION_SPINNER_DISPLAYED:14,LATENCY_ACTION_PLAYABILITY_CHECK:10,LATENCY_ACTION_PROCESS:9,LATENCY_ACTION_APP_STARTUP:5,LATENCY_ACTION_ABANDONED_DIRECT_PLAYBACK:154,LATENCY_ACTION_SHARE_VIDEO:153,LATENCY_ACTION_AD_TO_VIDEO_INT:152,LATENCY_ACTION_ABANDONED_BROWSE:151,LATENCY_ACTION_PLAYER_ROTATION:150,LATENCY_ACTION_SHOPPING_IN_APP:124,
LATENCY_ACTION_PLAYER_ATTESTATION:121,LATENCY_ACTION_PLAYER_SEEK:119,LATENCY_ACTION_SUPER_STICKER_BUY_FLOW:114,LATENCY_ACTION_BLOCKS_PERFORMANCE:148,LATENCY_ACTION_ASSISTANT_QUERY:138,LATENCY_ACTION_ASSISTANT_SETTINGS:137,LATENCY_ACTION_ENTITY_KEY_DESERIALIZATION_PERF:129,LATENCY_ACTION_ENTITY_KEY_SERIALIZATION_PERF:128,LATENCY_ACTION_PROOF_OF_ORIGIN_TOKEN_CREATE:127,LATENCY_ACTION_EMBEDS_SDK_INITIALIZATION:123,LATENCY_ACTION_NETWORKLESS_PERFORMANCE:122,LATENCY_ACTION_DOWNLOADS_EXPANSION:133,LATENCY_ACTION_ENTITY_TRANSFORM:131,
LATENCY_ACTION_DOWNLOADS_COMPATIBILITY_LAYER:96,LATENCY_ACTION_EMBEDS_SET_VIDEO:95,LATENCY_ACTION_SETTINGS:93,LATENCY_ACTION_ABANDONED_STARTUP:81,LATENCY_ACTION_MEDIA_BROWSER_ALARM_PLAY:80,LATENCY_ACTION_MEDIA_BROWSER_SEARCH:79,LATENCY_ACTION_MEDIA_BROWSER_LOAD_TREE:78,LATENCY_ACTION_WHO_IS_WATCHING:77,LATENCY_ACTION_CAST_LOAD_BY_ENTITY_TO_WATCH:76,LATENCY_ACTION_LITE_SWITCH_ACCOUNT:73,LATENCY_ACTION_ELEMENTS_PERFORMANCE:70,LATENCY_ACTION_LOCATION_SIGNAL_COLLECTION:69,LATENCY_ACTION_MODIFY_CHANNEL_NOTIFICATION:65,
LATENCY_ACTION_OFFLINE_STORE_START:61,LATENCY_ACTION_REEL_EDITOR:58,LATENCY_ACTION_CHANNEL_SUBSCRIBE:56,LATENCY_ACTION_CHANNEL_PREVIEW:55,LATENCY_ACTION_PREFETCH:52,LATENCY_ACTION_ABANDONED_WATCH:45,LATENCY_ACTION_LOAD_COMMENT_REPLIES:26,LATENCY_ACTION_LOAD_COMMENTS:25,LATENCY_ACTION_EDIT_COMMENT:24,LATENCY_ACTION_NEW_COMMENT:23,LATENCY_ACTION_OFFLINE_SHARING_RECEIVER_PAIRING:19,LATENCY_ACTION_EMBED:18,LATENCY_ACTION_MDX_LAUNCH:15,LATENCY_ACTION_RESOLVE_URL:13,LATENCY_ACTION_CAST_SPLASH:149,LATENCY_ACTION_MDX_CAST:120,
LATENCY_ACTION_MDX_COMMAND:12,LATENCY_ACTION_REEL_SELECT_SEGMENT:136,LATENCY_ACTION_ACCELERATED_EFFECTS:145,LATENCY_ACTION_UPLOAD_AUDIO_MIXER:147,LATENCY_ACTION_SHORTS_SEG_IMP_TRANSCODING:146,LATENCY_ACTION_SHORTS_AUDIO_PICKER_PLAYBACK:130,LATENCY_ACTION_SHORTS_WAVEFORM_DOWNLOAD:125,LATENCY_ACTION_SHORTS_VIDEO_INGESTION:155,LATENCY_ACTION_SHORTS_GALLERY:107,LATENCY_ACTION_SHORTS_TRIM:105,LATENCY_ACTION_SHORTS_EDIT:104,LATENCY_ACTION_SHORTS_CAMERA:103,LATENCY_ACTION_PARENT_TOOLS_DASHBOARD:102,LATENCY_ACTION_PARENT_TOOLS_COLLECTION:101,
LATENCY_ACTION_MUSIC_LOAD_RECOMMENDED_MEDIA_ITEMS:116,LATENCY_ACTION_MUSIC_LOAD_MEDIA_ITEMS:115,LATENCY_ACTION_MUSIC_ALBUM_DETAIL:72,LATENCY_ACTION_MUSIC_PLAYLIST_DETAIL:71,LATENCY_ACTION_CHIPS:68,LATENCY_ACTION_SEARCH_ZERO_STATE:67,LATENCY_ACTION_LIVE_PAGINATION:117,LATENCY_ACTION_LIVE:20,LATENCY_ACTION_PREBUFFER:40,LATENCY_ACTION_TENX:39,LATENCY_ACTION_KIDS_PROFILE_SETTINGS:94,LATENCY_ACTION_KIDS_WATCH_IT_AGAIN:92,LATENCY_ACTION_KIDS_SECRET_CODE:91,LATENCY_ACTION_KIDS_PARENT_PROFILE_SETTINGS:89,
LATENCY_ACTION_KIDS_ONBOARDING:88,LATENCY_ACTION_KIDS_VOICE_SEARCH:82,LATENCY_ACTION_KIDS_CURATED_COLLECTION:62,LATENCY_ACTION_KIDS_LIBRARY:53,LATENCY_ACTION_CREATOR_VIDEO_TRANSLATIONS:38,LATENCY_ACTION_CREATOR_VIDEO_MONETIZATION:74,LATENCY_ACTION_CREATOR_VIDEO_LIVE_STREAMING:141,LATENCY_ACTION_CREATOR_VIDEO_LIVE_SETTINGS:142,LATENCY_ACTION_CREATOR_VIDEO_EDITOR_ASYNC:51,LATENCY_ACTION_CREATOR_VIDEO_EDITOR:50,LATENCY_ACTION_CREATOR_VIDEO_EDIT:36,LATENCY_ACTION_CREATOR_VIDEO_COMMENTS:34,LATENCY_ACTION_CREATOR_VIDEO_ANALYTICS:33,
LATENCY_ACTION_CREATOR_POST_LIST:112,LATENCY_ACTION_CREATOR_POST_EDIT:110,LATENCY_ACTION_CREATOR_POST_COMMENTS:111,LATENCY_ACTION_CREATOR_LIVE_STREAMING:108,LATENCY_ACTION_CREATOR_DIALOG_UPLOADS:86,LATENCY_ACTION_CREATOR_DIALOG_COPYRIGHT_STRIKES:87,LATENCY_ACTION_CREATOR_CHANNEL_VIDEOS:32,LATENCY_ACTION_CREATOR_CHANNEL_TRANSLATIONS:48,LATENCY_ACTION_CREATOR_CHANNEL_PLAYLISTS:139,LATENCY_ACTION_CREATOR_CHANNEL_MUSIC:99,LATENCY_ACTION_CREATOR_CHANNEL_MONETIZATION:43,LATENCY_ACTION_CREATOR_CHANNEL_EDITING:113,
LATENCY_ACTION_CREATOR_CHANNEL_DASHBOARD:49,LATENCY_ACTION_CREATOR_CHANNEL_COPYRIGHT:44,LATENCY_ACTION_CREATOR_CHANNEL_COMMENTS:66,LATENCY_ACTION_CREATOR_CHANNEL_ANALYTICS:31,LATENCY_ACTION_CREATOR_ARTIST_PROFILE:85,LATENCY_ACTION_CREATOR_ARTIST_CONCERTS:84,LATENCY_ACTION_CREATOR_ARTIST_ANALYTICS:83,LATENCY_ACTION_CREATOR_ANALYTICS_EXPLORE:140,LATENCY_ACTION_STORYBOARD_THUMBNAILS:118,LATENCY_ACTION_SEARCH_THUMBNAILS:59,LATENCY_ACTION_ON_DEVICE_MODEL_DOWNLOAD:54,LATENCY_ACTION_VOICE_ASSISTANT:47,LATENCY_ACTION_SEARCH_UI:35,
LATENCY_ACTION_SUGGEST:30,LATENCY_ACTION_AUTO_SEARCH:126,LATENCY_ACTION_DOWNLOADS:98,LATENCY_ACTION_EXPLORE:75,LATENCY_ACTION_VIDEO_LIST:63,LATENCY_ACTION_HOME_RESUME:60,LATENCY_ACTION_SUBSCRIPTIONS_LIST:57,LATENCY_ACTION_THUMBNAIL_LOAD:42,LATENCY_ACTION_FIRST_THUMBNAIL_LOAD:29,LATENCY_ACTION_SUBSCRIPTIONS_FEED:109,LATENCY_ACTION_SUBSCRIPTIONS:28,LATENCY_ACTION_TRENDING:27,LATENCY_ACTION_LIBRARY:21,LATENCY_ACTION_VIDEO_THUMBNAIL:8,LATENCY_ACTION_SHOW_MORE:7,LATENCY_ACTION_VIDEO_PREVIEW:6,LATENCY_ACTION_PREBUFFER_VIDEO:144,
LATENCY_ACTION_PREFETCH_VIDEO:143,LATENCY_ACTION_DIRECT_PLAYBACK:132,LATENCY_ACTION_REEL_WATCH:41,LATENCY_ACTION_AD_TO_AD:22,LATENCY_ACTION_VIDEO_TO_AD:17,LATENCY_ACTION_AD_TO_VIDEO:16,LATENCY_ACTION_ONBOARDING:135,LATENCY_ACTION_LOGIN:97,LATENCY_ACTION_BROWSE:11,LATENCY_ACTION_CHANNELS:4,LATENCY_ACTION_WATCH:3,LATENCY_ACTION_RESULTS:2,LATENCY_ACTION_HOME:1,LATENCY_ACTION_STARTUP:106,LATENCY_ACTION_UNKNOWN:0};W[W.LATENCY_ACTION_KIDS_PROFILE_SWITCHER]="LATENCY_ACTION_KIDS_PROFILE_SWITCHER";
W[W.LATENCY_ACTION_OFFLINE_THUMBNAIL_TRANSFER]="LATENCY_ACTION_OFFLINE_THUMBNAIL_TRANSFER";W[W.LATENCY_ACTION_CREATOR_VIDEO_VIDEO_EDITOR_ASYNC]="LATENCY_ACTION_CREATOR_VIDEO_VIDEO_EDITOR_ASYNC";W[W.LATENCY_ACTION_CREATOR_VIDEO_VIDEO_EDITOR]="LATENCY_ACTION_CREATOR_VIDEO_VIDEO_EDITOR";W[W.LATENCY_ACTION_SPINNER_DISPLAYED]="LATENCY_ACTION_SPINNER_DISPLAYED";W[W.LATENCY_ACTION_PLAYABILITY_CHECK]="LATENCY_ACTION_PLAYABILITY_CHECK";W[W.LATENCY_ACTION_PROCESS]="LATENCY_ACTION_PROCESS";
W[W.LATENCY_ACTION_APP_STARTUP]="LATENCY_ACTION_APP_STARTUP";W[W.LATENCY_ACTION_ABANDONED_DIRECT_PLAYBACK]="LATENCY_ACTION_ABANDONED_DIRECT_PLAYBACK";W[W.LATENCY_ACTION_SHARE_VIDEO]="LATENCY_ACTION_SHARE_VIDEO";W[W.LATENCY_ACTION_AD_TO_VIDEO_INT]="LATENCY_ACTION_AD_TO_VIDEO_INT";W[W.LATENCY_ACTION_ABANDONED_BROWSE]="LATENCY_ACTION_ABANDONED_BROWSE";W[W.LATENCY_ACTION_PLAYER_ROTATION]="LATENCY_ACTION_PLAYER_ROTATION";W[W.LATENCY_ACTION_SHOPPING_IN_APP]="LATENCY_ACTION_SHOPPING_IN_APP";
W[W.LATENCY_ACTION_PLAYER_ATTESTATION]="LATENCY_ACTION_PLAYER_ATTESTATION";W[W.LATENCY_ACTION_PLAYER_SEEK]="LATENCY_ACTION_PLAYER_SEEK";W[W.LATENCY_ACTION_SUPER_STICKER_BUY_FLOW]="LATENCY_ACTION_SUPER_STICKER_BUY_FLOW";W[W.LATENCY_ACTION_BLOCKS_PERFORMANCE]="LATENCY_ACTION_BLOCKS_PERFORMANCE";W[W.LATENCY_ACTION_ASSISTANT_QUERY]="LATENCY_ACTION_ASSISTANT_QUERY";W[W.LATENCY_ACTION_ASSISTANT_SETTINGS]="LATENCY_ACTION_ASSISTANT_SETTINGS";W[W.LATENCY_ACTION_ENTITY_KEY_DESERIALIZATION_PERF]="LATENCY_ACTION_ENTITY_KEY_DESERIALIZATION_PERF";
W[W.LATENCY_ACTION_ENTITY_KEY_SERIALIZATION_PERF]="LATENCY_ACTION_ENTITY_KEY_SERIALIZATION_PERF";W[W.LATENCY_ACTION_PROOF_OF_ORIGIN_TOKEN_CREATE]="LATENCY_ACTION_PROOF_OF_ORIGIN_TOKEN_CREATE";W[W.LATENCY_ACTION_EMBEDS_SDK_INITIALIZATION]="LATENCY_ACTION_EMBEDS_SDK_INITIALIZATION";W[W.LATENCY_ACTION_NETWORKLESS_PERFORMANCE]="LATENCY_ACTION_NETWORKLESS_PERFORMANCE";W[W.LATENCY_ACTION_DOWNLOADS_EXPANSION]="LATENCY_ACTION_DOWNLOADS_EXPANSION";W[W.LATENCY_ACTION_ENTITY_TRANSFORM]="LATENCY_ACTION_ENTITY_TRANSFORM";
W[W.LATENCY_ACTION_DOWNLOADS_COMPATIBILITY_LAYER]="LATENCY_ACTION_DOWNLOADS_COMPATIBILITY_LAYER";W[W.LATENCY_ACTION_EMBEDS_SET_VIDEO]="LATENCY_ACTION_EMBEDS_SET_VIDEO";W[W.LATENCY_ACTION_SETTINGS]="LATENCY_ACTION_SETTINGS";W[W.LATENCY_ACTION_ABANDONED_STARTUP]="LATENCY_ACTION_ABANDONED_STARTUP";W[W.LATENCY_ACTION_MEDIA_BROWSER_ALARM_PLAY]="LATENCY_ACTION_MEDIA_BROWSER_ALARM_PLAY";W[W.LATENCY_ACTION_MEDIA_BROWSER_SEARCH]="LATENCY_ACTION_MEDIA_BROWSER_SEARCH";
W[W.LATENCY_ACTION_MEDIA_BROWSER_LOAD_TREE]="LATENCY_ACTION_MEDIA_BROWSER_LOAD_TREE";W[W.LATENCY_ACTION_WHO_IS_WATCHING]="LATENCY_ACTION_WHO_IS_WATCHING";W[W.LATENCY_ACTION_CAST_LOAD_BY_ENTITY_TO_WATCH]="LATENCY_ACTION_CAST_LOAD_BY_ENTITY_TO_WATCH";W[W.LATENCY_ACTION_LITE_SWITCH_ACCOUNT]="LATENCY_ACTION_LITE_SWITCH_ACCOUNT";W[W.LATENCY_ACTION_ELEMENTS_PERFORMANCE]="LATENCY_ACTION_ELEMENTS_PERFORMANCE";W[W.LATENCY_ACTION_LOCATION_SIGNAL_COLLECTION]="LATENCY_ACTION_LOCATION_SIGNAL_COLLECTION";
W[W.LATENCY_ACTION_MODIFY_CHANNEL_NOTIFICATION]="LATENCY_ACTION_MODIFY_CHANNEL_NOTIFICATION";W[W.LATENCY_ACTION_OFFLINE_STORE_START]="LATENCY_ACTION_OFFLINE_STORE_START";W[W.LATENCY_ACTION_REEL_EDITOR]="LATENCY_ACTION_REEL_EDITOR";W[W.LATENCY_ACTION_CHANNEL_SUBSCRIBE]="LATENCY_ACTION_CHANNEL_SUBSCRIBE";W[W.LATENCY_ACTION_CHANNEL_PREVIEW]="LATENCY_ACTION_CHANNEL_PREVIEW";W[W.LATENCY_ACTION_PREFETCH]="LATENCY_ACTION_PREFETCH";W[W.LATENCY_ACTION_ABANDONED_WATCH]="LATENCY_ACTION_ABANDONED_WATCH";
W[W.LATENCY_ACTION_LOAD_COMMENT_REPLIES]="LATENCY_ACTION_LOAD_COMMENT_REPLIES";W[W.LATENCY_ACTION_LOAD_COMMENTS]="LATENCY_ACTION_LOAD_COMMENTS";W[W.LATENCY_ACTION_EDIT_COMMENT]="LATENCY_ACTION_EDIT_COMMENT";W[W.LATENCY_ACTION_NEW_COMMENT]="LATENCY_ACTION_NEW_COMMENT";W[W.LATENCY_ACTION_OFFLINE_SHARING_RECEIVER_PAIRING]="LATENCY_ACTION_OFFLINE_SHARING_RECEIVER_PAIRING";W[W.LATENCY_ACTION_EMBED]="LATENCY_ACTION_EMBED";W[W.LATENCY_ACTION_MDX_LAUNCH]="LATENCY_ACTION_MDX_LAUNCH";
W[W.LATENCY_ACTION_RESOLVE_URL]="LATENCY_ACTION_RESOLVE_URL";W[W.LATENCY_ACTION_CAST_SPLASH]="LATENCY_ACTION_CAST_SPLASH";W[W.LATENCY_ACTION_MDX_CAST]="LATENCY_ACTION_MDX_CAST";W[W.LATENCY_ACTION_MDX_COMMAND]="LATENCY_ACTION_MDX_COMMAND";W[W.LATENCY_ACTION_REEL_SELECT_SEGMENT]="LATENCY_ACTION_REEL_SELECT_SEGMENT";W[W.LATENCY_ACTION_ACCELERATED_EFFECTS]="LATENCY_ACTION_ACCELERATED_EFFECTS";W[W.LATENCY_ACTION_UPLOAD_AUDIO_MIXER]="LATENCY_ACTION_UPLOAD_AUDIO_MIXER";
W[W.LATENCY_ACTION_SHORTS_SEG_IMP_TRANSCODING]="LATENCY_ACTION_SHORTS_SEG_IMP_TRANSCODING";W[W.LATENCY_ACTION_SHORTS_AUDIO_PICKER_PLAYBACK]="LATENCY_ACTION_SHORTS_AUDIO_PICKER_PLAYBACK";W[W.LATENCY_ACTION_SHORTS_WAVEFORM_DOWNLOAD]="LATENCY_ACTION_SHORTS_WAVEFORM_DOWNLOAD";W[W.LATENCY_ACTION_SHORTS_VIDEO_INGESTION]="LATENCY_ACTION_SHORTS_VIDEO_INGESTION";W[W.LATENCY_ACTION_SHORTS_GALLERY]="LATENCY_ACTION_SHORTS_GALLERY";W[W.LATENCY_ACTION_SHORTS_TRIM]="LATENCY_ACTION_SHORTS_TRIM";
W[W.LATENCY_ACTION_SHORTS_EDIT]="LATENCY_ACTION_SHORTS_EDIT";W[W.LATENCY_ACTION_SHORTS_CAMERA]="LATENCY_ACTION_SHORTS_CAMERA";W[W.LATENCY_ACTION_PARENT_TOOLS_DASHBOARD]="LATENCY_ACTION_PARENT_TOOLS_DASHBOARD";W[W.LATENCY_ACTION_PARENT_TOOLS_COLLECTION]="LATENCY_ACTION_PARENT_TOOLS_COLLECTION";W[W.LATENCY_ACTION_MUSIC_LOAD_RECOMMENDED_MEDIA_ITEMS]="LATENCY_ACTION_MUSIC_LOAD_RECOMMENDED_MEDIA_ITEMS";W[W.LATENCY_ACTION_MUSIC_LOAD_MEDIA_ITEMS]="LATENCY_ACTION_MUSIC_LOAD_MEDIA_ITEMS";
W[W.LATENCY_ACTION_MUSIC_ALBUM_DETAIL]="LATENCY_ACTION_MUSIC_ALBUM_DETAIL";W[W.LATENCY_ACTION_MUSIC_PLAYLIST_DETAIL]="LATENCY_ACTION_MUSIC_PLAYLIST_DETAIL";W[W.LATENCY_ACTION_CHIPS]="LATENCY_ACTION_CHIPS";W[W.LATENCY_ACTION_SEARCH_ZERO_STATE]="LATENCY_ACTION_SEARCH_ZERO_STATE";W[W.LATENCY_ACTION_LIVE_PAGINATION]="LATENCY_ACTION_LIVE_PAGINATION";W[W.LATENCY_ACTION_LIVE]="LATENCY_ACTION_LIVE";W[W.LATENCY_ACTION_PREBUFFER]="LATENCY_ACTION_PREBUFFER";W[W.LATENCY_ACTION_TENX]="LATENCY_ACTION_TENX";
W[W.LATENCY_ACTION_KIDS_PROFILE_SETTINGS]="LATENCY_ACTION_KIDS_PROFILE_SETTINGS";W[W.LATENCY_ACTION_KIDS_WATCH_IT_AGAIN]="LATENCY_ACTION_KIDS_WATCH_IT_AGAIN";W[W.LATENCY_ACTION_KIDS_SECRET_CODE]="LATENCY_ACTION_KIDS_SECRET_CODE";W[W.LATENCY_ACTION_KIDS_PARENT_PROFILE_SETTINGS]="LATENCY_ACTION_KIDS_PARENT_PROFILE_SETTINGS";W[W.LATENCY_ACTION_KIDS_ONBOARDING]="LATENCY_ACTION_KIDS_ONBOARDING";W[W.LATENCY_ACTION_KIDS_VOICE_SEARCH]="LATENCY_ACTION_KIDS_VOICE_SEARCH";
W[W.LATENCY_ACTION_KIDS_CURATED_COLLECTION]="LATENCY_ACTION_KIDS_CURATED_COLLECTION";W[W.LATENCY_ACTION_KIDS_LIBRARY]="LATENCY_ACTION_KIDS_LIBRARY";W[W.LATENCY_ACTION_CREATOR_VIDEO_TRANSLATIONS]="LATENCY_ACTION_CREATOR_VIDEO_TRANSLATIONS";W[W.LATENCY_ACTION_CREATOR_VIDEO_MONETIZATION]="LATENCY_ACTION_CREATOR_VIDEO_MONETIZATION";W[W.LATENCY_ACTION_CREATOR_VIDEO_LIVE_STREAMING]="LATENCY_ACTION_CREATOR_VIDEO_LIVE_STREAMING";W[W.LATENCY_ACTION_CREATOR_VIDEO_LIVE_SETTINGS]="LATENCY_ACTION_CREATOR_VIDEO_LIVE_SETTINGS";
W[W.LATENCY_ACTION_CREATOR_VIDEO_EDITOR_ASYNC]="LATENCY_ACTION_CREATOR_VIDEO_EDITOR_ASYNC";W[W.LATENCY_ACTION_CREATOR_VIDEO_EDITOR]="LATENCY_ACTION_CREATOR_VIDEO_EDITOR";W[W.LATENCY_ACTION_CREATOR_VIDEO_EDIT]="LATENCY_ACTION_CREATOR_VIDEO_EDIT";W[W.LATENCY_ACTION_CREATOR_VIDEO_COMMENTS]="LATENCY_ACTION_CREATOR_VIDEO_COMMENTS";W[W.LATENCY_ACTION_CREATOR_VIDEO_ANALYTICS]="LATENCY_ACTION_CREATOR_VIDEO_ANALYTICS";W[W.LATENCY_ACTION_CREATOR_POST_LIST]="LATENCY_ACTION_CREATOR_POST_LIST";
W[W.LATENCY_ACTION_CREATOR_POST_EDIT]="LATENCY_ACTION_CREATOR_POST_EDIT";W[W.LATENCY_ACTION_CREATOR_POST_COMMENTS]="LATENCY_ACTION_CREATOR_POST_COMMENTS";W[W.LATENCY_ACTION_CREATOR_LIVE_STREAMING]="LATENCY_ACTION_CREATOR_LIVE_STREAMING";W[W.LATENCY_ACTION_CREATOR_DIALOG_UPLOADS]="LATENCY_ACTION_CREATOR_DIALOG_UPLOADS";W[W.LATENCY_ACTION_CREATOR_DIALOG_COPYRIGHT_STRIKES]="LATENCY_ACTION_CREATOR_DIALOG_COPYRIGHT_STRIKES";W[W.LATENCY_ACTION_CREATOR_CHANNEL_VIDEOS]="LATENCY_ACTION_CREATOR_CHANNEL_VIDEOS";
W[W.LATENCY_ACTION_CREATOR_CHANNEL_TRANSLATIONS]="LATENCY_ACTION_CREATOR_CHANNEL_TRANSLATIONS";W[W.LATENCY_ACTION_CREATOR_CHANNEL_PLAYLISTS]="LATENCY_ACTION_CREATOR_CHANNEL_PLAYLISTS";W[W.LATENCY_ACTION_CREATOR_CHANNEL_MUSIC]="LATENCY_ACTION_CREATOR_CHANNEL_MUSIC";W[W.LATENCY_ACTION_CREATOR_CHANNEL_MONETIZATION]="LATENCY_ACTION_CREATOR_CHANNEL_MONETIZATION";W[W.LATENCY_ACTION_CREATOR_CHANNEL_EDITING]="LATENCY_ACTION_CREATOR_CHANNEL_EDITING";W[W.LATENCY_ACTION_CREATOR_CHANNEL_DASHBOARD]="LATENCY_ACTION_CREATOR_CHANNEL_DASHBOARD";
W[W.LATENCY_ACTION_CREATOR_CHANNEL_COPYRIGHT]="LATENCY_ACTION_CREATOR_CHANNEL_COPYRIGHT";W[W.LATENCY_ACTION_CREATOR_CHANNEL_COMMENTS]="LATENCY_ACTION_CREATOR_CHANNEL_COMMENTS";W[W.LATENCY_ACTION_CREATOR_CHANNEL_ANALYTICS]="LATENCY_ACTION_CREATOR_CHANNEL_ANALYTICS";W[W.LATENCY_ACTION_CREATOR_ARTIST_PROFILE]="LATENCY_ACTION_CREATOR_ARTIST_PROFILE";W[W.LATENCY_ACTION_CREATOR_ARTIST_CONCERTS]="LATENCY_ACTION_CREATOR_ARTIST_CONCERTS";W[W.LATENCY_ACTION_CREATOR_ARTIST_ANALYTICS]="LATENCY_ACTION_CREATOR_ARTIST_ANALYTICS";
W[W.LATENCY_ACTION_CREATOR_ANALYTICS_EXPLORE]="LATENCY_ACTION_CREATOR_ANALYTICS_EXPLORE";W[W.LATENCY_ACTION_STORYBOARD_THUMBNAILS]="LATENCY_ACTION_STORYBOARD_THUMBNAILS";W[W.LATENCY_ACTION_SEARCH_THUMBNAILS]="LATENCY_ACTION_SEARCH_THUMBNAILS";W[W.LATENCY_ACTION_ON_DEVICE_MODEL_DOWNLOAD]="LATENCY_ACTION_ON_DEVICE_MODEL_DOWNLOAD";W[W.LATENCY_ACTION_VOICE_ASSISTANT]="LATENCY_ACTION_VOICE_ASSISTANT";W[W.LATENCY_ACTION_SEARCH_UI]="LATENCY_ACTION_SEARCH_UI";W[W.LATENCY_ACTION_SUGGEST]="LATENCY_ACTION_SUGGEST";
W[W.LATENCY_ACTION_AUTO_SEARCH]="LATENCY_ACTION_AUTO_SEARCH";W[W.LATENCY_ACTION_DOWNLOADS]="LATENCY_ACTION_DOWNLOADS";W[W.LATENCY_ACTION_EXPLORE]="LATENCY_ACTION_EXPLORE";W[W.LATENCY_ACTION_VIDEO_LIST]="LATENCY_ACTION_VIDEO_LIST";W[W.LATENCY_ACTION_HOME_RESUME]="LATENCY_ACTION_HOME_RESUME";W[W.LATENCY_ACTION_SUBSCRIPTIONS_LIST]="LATENCY_ACTION_SUBSCRIPTIONS_LIST";W[W.LATENCY_ACTION_THUMBNAIL_LOAD]="LATENCY_ACTION_THUMBNAIL_LOAD";W[W.LATENCY_ACTION_FIRST_THUMBNAIL_LOAD]="LATENCY_ACTION_FIRST_THUMBNAIL_LOAD";
W[W.LATENCY_ACTION_SUBSCRIPTIONS_FEED]="LATENCY_ACTION_SUBSCRIPTIONS_FEED";W[W.LATENCY_ACTION_SUBSCRIPTIONS]="LATENCY_ACTION_SUBSCRIPTIONS";W[W.LATENCY_ACTION_TRENDING]="LATENCY_ACTION_TRENDING";W[W.LATENCY_ACTION_LIBRARY]="LATENCY_ACTION_LIBRARY";W[W.LATENCY_ACTION_VIDEO_THUMBNAIL]="LATENCY_ACTION_VIDEO_THUMBNAIL";W[W.LATENCY_ACTION_SHOW_MORE]="LATENCY_ACTION_SHOW_MORE";W[W.LATENCY_ACTION_VIDEO_PREVIEW]="LATENCY_ACTION_VIDEO_PREVIEW";W[W.LATENCY_ACTION_PREBUFFER_VIDEO]="LATENCY_ACTION_PREBUFFER_VIDEO";
W[W.LATENCY_ACTION_PREFETCH_VIDEO]="LATENCY_ACTION_PREFETCH_VIDEO";W[W.LATENCY_ACTION_DIRECT_PLAYBACK]="LATENCY_ACTION_DIRECT_PLAYBACK";W[W.LATENCY_ACTION_REEL_WATCH]="LATENCY_ACTION_REEL_WATCH";W[W.LATENCY_ACTION_AD_TO_AD]="LATENCY_ACTION_AD_TO_AD";W[W.LATENCY_ACTION_VIDEO_TO_AD]="LATENCY_ACTION_VIDEO_TO_AD";W[W.LATENCY_ACTION_AD_TO_VIDEO]="LATENCY_ACTION_AD_TO_VIDEO";W[W.LATENCY_ACTION_ONBOARDING]="LATENCY_ACTION_ONBOARDING";W[W.LATENCY_ACTION_LOGIN]="LATENCY_ACTION_LOGIN";
W[W.LATENCY_ACTION_BROWSE]="LATENCY_ACTION_BROWSE";W[W.LATENCY_ACTION_CHANNELS]="LATENCY_ACTION_CHANNELS";W[W.LATENCY_ACTION_WATCH]="LATENCY_ACTION_WATCH";W[W.LATENCY_ACTION_RESULTS]="LATENCY_ACTION_RESULTS";W[W.LATENCY_ACTION_HOME]="LATENCY_ACTION_HOME";W[W.LATENCY_ACTION_STARTUP]="LATENCY_ACTION_STARTUP";W[W.LATENCY_ACTION_UNKNOWN]="LATENCY_ACTION_UNKNOWN";var Mp={LATENCY_NETWORK_MOBILE:2,LATENCY_NETWORK_WIFI:1,LATENCY_NETWORK_UNKNOWN:0};Mp[Mp.LATENCY_NETWORK_MOBILE]="LATENCY_NETWORK_MOBILE";
Mp[Mp.LATENCY_NETWORK_WIFI]="LATENCY_NETWORK_WIFI";Mp[Mp.LATENCY_NETWORK_UNKNOWN]="LATENCY_NETWORK_UNKNOWN";var X={CONN_INVALID:31,CONN_CELLULAR_5G_NSA:12,CONN_CELLULAR_5G_SA:11,CONN_WIFI_METERED:10,CONN_CELLULAR_5G:9,CONN_DISCO:8,CONN_CELLULAR_UNKNOWN:7,CONN_CELLULAR_4G:6,CONN_CELLULAR_3G:5,CONN_CELLULAR_2G:4,CONN_WIFI:3,CONN_NONE:2,CONN_UNKNOWN:1,CONN_DEFAULT:0};X[X.CONN_INVALID]="CONN_INVALID";X[X.CONN_CELLULAR_5G_NSA]="CONN_CELLULAR_5G_NSA";X[X.CONN_CELLULAR_5G_SA]="CONN_CELLULAR_5G_SA";
X[X.CONN_WIFI_METERED]="CONN_WIFI_METERED";X[X.CONN_CELLULAR_5G]="CONN_CELLULAR_5G";X[X.CONN_DISCO]="CONN_DISCO";X[X.CONN_CELLULAR_UNKNOWN]="CONN_CELLULAR_UNKNOWN";X[X.CONN_CELLULAR_4G]="CONN_CELLULAR_4G";X[X.CONN_CELLULAR_3G]="CONN_CELLULAR_3G";X[X.CONN_CELLULAR_2G]="CONN_CELLULAR_2G";X[X.CONN_WIFI]="CONN_WIFI";X[X.CONN_NONE]="CONN_NONE";X[X.CONN_UNKNOWN]="CONN_UNKNOWN";X[X.CONN_DEFAULT]="CONN_DEFAULT";
var Y={DETAILED_NETWORK_TYPE_NR_NSA:126,DETAILED_NETWORK_TYPE_NR_SA:125,DETAILED_NETWORK_TYPE_INTERNAL_WIFI_IMPAIRED:124,DETAILED_NETWORK_TYPE_APP_WIFI_HOTSPOT:123,DETAILED_NETWORK_TYPE_DISCONNECTED:122,DETAILED_NETWORK_TYPE_NON_MOBILE_UNKNOWN:121,DETAILED_NETWORK_TYPE_MOBILE_UNKNOWN:120,DETAILED_NETWORK_TYPE_WIMAX:119,DETAILED_NETWORK_TYPE_ETHERNET:118,DETAILED_NETWORK_TYPE_BLUETOOTH:117,DETAILED_NETWORK_TYPE_WIFI:116,DETAILED_NETWORK_TYPE_LTE:115,DETAILED_NETWORK_TYPE_HSPAP:114,DETAILED_NETWORK_TYPE_EHRPD:113,
DETAILED_NETWORK_TYPE_EVDO_B:112,DETAILED_NETWORK_TYPE_UMTS:111,DETAILED_NETWORK_TYPE_IDEN:110,DETAILED_NETWORK_TYPE_HSUPA:109,DETAILED_NETWORK_TYPE_HSPA:108,DETAILED_NETWORK_TYPE_HSDPA:107,DETAILED_NETWORK_TYPE_EVDO_A:106,DETAILED_NETWORK_TYPE_EVDO_0:105,DETAILED_NETWORK_TYPE_CDMA:104,DETAILED_NETWORK_TYPE_1_X_RTT:103,DETAILED_NETWORK_TYPE_GPRS:102,DETAILED_NETWORK_TYPE_EDGE:101,DETAILED_NETWORK_TYPE_UNKNOWN:0};Y[Y.DETAILED_NETWORK_TYPE_NR_NSA]="DETAILED_NETWORK_TYPE_NR_NSA";
Y[Y.DETAILED_NETWORK_TYPE_NR_SA]="DETAILED_NETWORK_TYPE_NR_SA";Y[Y.DETAILED_NETWORK_TYPE_INTERNAL_WIFI_IMPAIRED]="DETAILED_NETWORK_TYPE_INTERNAL_WIFI_IMPAIRED";Y[Y.DETAILED_NETWORK_TYPE_APP_WIFI_HOTSPOT]="DETAILED_NETWORK_TYPE_APP_WIFI_HOTSPOT";Y[Y.DETAILED_NETWORK_TYPE_DISCONNECTED]="DETAILED_NETWORK_TYPE_DISCONNECTED";Y[Y.DETAILED_NETWORK_TYPE_NON_MOBILE_UNKNOWN]="DETAILED_NETWORK_TYPE_NON_MOBILE_UNKNOWN";Y[Y.DETAILED_NETWORK_TYPE_MOBILE_UNKNOWN]="DETAILED_NETWORK_TYPE_MOBILE_UNKNOWN";
Y[Y.DETAILED_NETWORK_TYPE_WIMAX]="DETAILED_NETWORK_TYPE_WIMAX";Y[Y.DETAILED_NETWORK_TYPE_ETHERNET]="DETAILED_NETWORK_TYPE_ETHERNET";Y[Y.DETAILED_NETWORK_TYPE_BLUETOOTH]="DETAILED_NETWORK_TYPE_BLUETOOTH";Y[Y.DETAILED_NETWORK_TYPE_WIFI]="DETAILED_NETWORK_TYPE_WIFI";Y[Y.DETAILED_NETWORK_TYPE_LTE]="DETAILED_NETWORK_TYPE_LTE";Y[Y.DETAILED_NETWORK_TYPE_HSPAP]="DETAILED_NETWORK_TYPE_HSPAP";Y[Y.DETAILED_NETWORK_TYPE_EHRPD]="DETAILED_NETWORK_TYPE_EHRPD";Y[Y.DETAILED_NETWORK_TYPE_EVDO_B]="DETAILED_NETWORK_TYPE_EVDO_B";
Y[Y.DETAILED_NETWORK_TYPE_UMTS]="DETAILED_NETWORK_TYPE_UMTS";Y[Y.DETAILED_NETWORK_TYPE_IDEN]="DETAILED_NETWORK_TYPE_IDEN";Y[Y.DETAILED_NETWORK_TYPE_HSUPA]="DETAILED_NETWORK_TYPE_HSUPA";Y[Y.DETAILED_NETWORK_TYPE_HSPA]="DETAILED_NETWORK_TYPE_HSPA";Y[Y.DETAILED_NETWORK_TYPE_HSDPA]="DETAILED_NETWORK_TYPE_HSDPA";Y[Y.DETAILED_NETWORK_TYPE_EVDO_A]="DETAILED_NETWORK_TYPE_EVDO_A";Y[Y.DETAILED_NETWORK_TYPE_EVDO_0]="DETAILED_NETWORK_TYPE_EVDO_0";Y[Y.DETAILED_NETWORK_TYPE_CDMA]="DETAILED_NETWORK_TYPE_CDMA";
Y[Y.DETAILED_NETWORK_TYPE_1_X_RTT]="DETAILED_NETWORK_TYPE_1_X_RTT";Y[Y.DETAILED_NETWORK_TYPE_GPRS]="DETAILED_NETWORK_TYPE_GPRS";Y[Y.DETAILED_NETWORK_TYPE_EDGE]="DETAILED_NETWORK_TYPE_EDGE";Y[Y.DETAILED_NETWORK_TYPE_UNKNOWN]="DETAILED_NETWORK_TYPE_UNKNOWN";var Np={LATENCY_PLAYER_RTSP:7,LATENCY_PLAYER_HTML5_INLINE:6,LATENCY_PLAYER_HTML5_FULLSCREEN:5,LATENCY_PLAYER_HTML5:4,LATENCY_PLAYER_FRAMEWORK:3,LATENCY_PLAYER_FLASH:2,LATENCY_PLAYER_EXO:1,LATENCY_PLAYER_UNKNOWN:0};Np[Np.LATENCY_PLAYER_RTSP]="LATENCY_PLAYER_RTSP";
Np[Np.LATENCY_PLAYER_HTML5_INLINE]="LATENCY_PLAYER_HTML5_INLINE";Np[Np.LATENCY_PLAYER_HTML5_FULLSCREEN]="LATENCY_PLAYER_HTML5_FULLSCREEN";Np[Np.LATENCY_PLAYER_HTML5]="LATENCY_PLAYER_HTML5";Np[Np.LATENCY_PLAYER_FRAMEWORK]="LATENCY_PLAYER_FRAMEWORK";Np[Np.LATENCY_PLAYER_FLASH]="LATENCY_PLAYER_FLASH";Np[Np.LATENCY_PLAYER_EXO]="LATENCY_PLAYER_EXO";Np[Np.LATENCY_PLAYER_UNKNOWN]="LATENCY_PLAYER_UNKNOWN";
var Op={LATENCY_AD_BREAK_TYPE_POSTROLL:3,LATENCY_AD_BREAK_TYPE_MIDROLL:2,LATENCY_AD_BREAK_TYPE_PREROLL:1,LATENCY_AD_BREAK_TYPE_UNKNOWN:0};Op[Op.LATENCY_AD_BREAK_TYPE_POSTROLL]="LATENCY_AD_BREAK_TYPE_POSTROLL";Op[Op.LATENCY_AD_BREAK_TYPE_MIDROLL]="LATENCY_AD_BREAK_TYPE_MIDROLL";Op[Op.LATENCY_AD_BREAK_TYPE_PREROLL]="LATENCY_AD_BREAK_TYPE_PREROLL";Op[Op.LATENCY_AD_BREAK_TYPE_UNKNOWN]="LATENCY_AD_BREAK_TYPE_UNKNOWN";var Pp={LATENCY_ACTION_ERROR_STARTUP_TIMEOUT:1,LATENCY_ACTION_ERROR_UNSPECIFIED:0};
Pp[Pp.LATENCY_ACTION_ERROR_STARTUP_TIMEOUT]="LATENCY_ACTION_ERROR_STARTUP_TIMEOUT";Pp[Pp.LATENCY_ACTION_ERROR_UNSPECIFIED]="LATENCY_ACTION_ERROR_UNSPECIFIED";var Qp={LIVE_STREAM_MODE_WINDOW:5,LIVE_STREAM_MODE_POST:4,LIVE_STREAM_MODE_LP:3,LIVE_STREAM_MODE_LIVE:2,LIVE_STREAM_MODE_DVR:1,LIVE_STREAM_MODE_UNKNOWN:0};Qp[Qp.LIVE_STREAM_MODE_WINDOW]="LIVE_STREAM_MODE_WINDOW";Qp[Qp.LIVE_STREAM_MODE_POST]="LIVE_STREAM_MODE_POST";Qp[Qp.LIVE_STREAM_MODE_LP]="LIVE_STREAM_MODE_LP";
Qp[Qp.LIVE_STREAM_MODE_LIVE]="LIVE_STREAM_MODE_LIVE";Qp[Qp.LIVE_STREAM_MODE_DVR]="LIVE_STREAM_MODE_DVR";Qp[Qp.LIVE_STREAM_MODE_UNKNOWN]="LIVE_STREAM_MODE_UNKNOWN";var Rp={VIDEO_STREAM_TYPE_VOD:3,VIDEO_STREAM_TYPE_DVR:2,VIDEO_STREAM_TYPE_LIVE:1,VIDEO_STREAM_TYPE_UNSPECIFIED:0};Rp[Rp.VIDEO_STREAM_TYPE_VOD]="VIDEO_STREAM_TYPE_VOD";Rp[Rp.VIDEO_STREAM_TYPE_DVR]="VIDEO_STREAM_TYPE_DVR";Rp[Rp.VIDEO_STREAM_TYPE_LIVE]="VIDEO_STREAM_TYPE_LIVE";Rp[Rp.VIDEO_STREAM_TYPE_UNSPECIFIED]="VIDEO_STREAM_TYPE_UNSPECIFIED";
var Sp={YT_IDB_TRANSACTION_TYPE_READ:2,YT_IDB_TRANSACTION_TYPE_WRITE:1,YT_IDB_TRANSACTION_TYPE_UNKNOWN:0};Sp[Sp.YT_IDB_TRANSACTION_TYPE_READ]="YT_IDB_TRANSACTION_TYPE_READ";Sp[Sp.YT_IDB_TRANSACTION_TYPE_WRITE]="YT_IDB_TRANSACTION_TYPE_WRITE";Sp[Sp.YT_IDB_TRANSACTION_TYPE_UNKNOWN]="YT_IDB_TRANSACTION_TYPE_UNKNOWN";var Tp={PLAYER_ROTATION_TYPE_PORTRAIT_TO_FULLSCREEN:2,PLAYER_ROTATION_TYPE_FULLSCREEN_TO_PORTRAIT:1,PLAYER_ROTATION_TYPE_UNKNOWN:0};Tp[Tp.PLAYER_ROTATION_TYPE_PORTRAIT_TO_FULLSCREEN]="PLAYER_ROTATION_TYPE_PORTRAIT_TO_FULLSCREEN";
Tp[Tp.PLAYER_ROTATION_TYPE_FULLSCREEN_TO_PORTRAIT]="PLAYER_ROTATION_TYPE_FULLSCREEN_TO_PORTRAIT";Tp[Tp.PLAYER_ROTATION_TYPE_UNKNOWN]="PLAYER_ROTATION_TYPE_UNKNOWN";var Up=x.ytLoggingGelSequenceIdObj_||{};z("ytLoggingGelSequenceIdObj_",Up,void 0);function Vp(a,b,c){c=void 0===c?{}:c;var d=Math.round(c.timestamp||O());F(a,1,d<Number.MAX_SAFE_INTEGER?d:0);var e=fi();d=new Zg;F(d,1,c.timestamp||!isFinite(e)?-1:e);if(K("log_sequence_info_on_gel_web")&&c.X){e=c.X;var f=Ej(e),g=new Yg;F(g,2,f);F(g,1,e);G(d,3,g);c.ub&&delete Up[c.X]}G(a,33,d);(c.mc?oj:jj)({endpoint:"log_event",payload:a,cttAuthInfo:c.cttAuthInfo,wa:c.wa},b)}
;function Wp(a,b){b=void 0===b?{}:b;var c=!1;B("ytLoggingEventsDefaultDisabled",!1)&&zn===zn&&(c=!0);Vp(a,c?null:zn,b)}
;function Xp(a,b,c){var d=new $g;Vc(d,72,a);c?Vp(d,c,b):Wp(d,b)}
function Yp(a,b,c){var d=new $g;Vc(d,73,a);c?Vp(d,c,b):Wp(d,b)}
function Zp(a,b,c){var d=new $g;Vc(d,78,a);c?Vp(d,c,b):Wp(d,b)}
function $p(a,b,c){var d=new $g;Vc(d,208,a);c?Vp(d,c,b):Wp(d,b)}
function aq(a,b,c){var d=new $g;Vc(d,156,a);c?Vp(d,c,b):Wp(d,b)}
function bq(a,b,c){var d=new $g;Vc(d,215,a);c?Vp(d,c,b):Wp(d,b)}
;var cq=x.ytLoggingLatencyUsageStats_||{};z("ytLoggingLatencyUsageStats_",cq,void 0);function dq(){this.h=0}
function eq(){dq.h||(dq.h=new dq);return dq.h}
dq.prototype.tick=function(a,b,c,d){fq(this,"tick_"+a+"_"+b)||(c={timestamp:c,cttAuthInfo:d},K("web_csi_via_jspb")?(d=new Xg,F(d,1,a),F(d,2,b),a=new $g,Vc(a,5,d),Wp(a,c)):wk("latencyActionTicked",{tickName:a,clientActionNonce:b},c))};
dq.prototype.info=function(a,b,c){var d=Object.keys(a).join("");fq(this,"info_"+d+"_"+b)||(a=Object.assign({},a),a.clientActionNonce=b,wk("latencyActionInfo",a,{cttAuthInfo:c}))};
dq.prototype.jspbInfo=function(a,b,c){for(var d="",e=0;e<a.toJSON().length;e++)void 0!==a.toJSON()[e]&&(d=0===e?d.concat(""+e):d.concat("_"+e));fq(this,"info_"+d+"_"+b)||(F(a,2,b),b={cttAuthInfo:c},c=new $g,Vc(c,7,a),Wp(c,b))};
dq.prototype.span=function(a,b,c){var d=Object.keys(a).join("");fq(this,"span_"+d+"_"+b)||(a.clientActionNonce=b,wk("latencyActionSpan",a,{cttAuthInfo:c}))};
function fq(a,b){cq[b]=cq[b]||{count:0};var c=cq[b];c.count++;c.time=O();a.h||(a.h=Vh(function(){var d=O(),e;for(e in cq)cq[e]&&6E4<d-cq[e].time&&delete cq[e];a&&(a.h=0)},5E3));
return 5<c.count?(6===c.count&&1>1E5*Math.random()&&(c=new N("CSI data exceeded logging limit with key",b.split("_")),0<=b.indexOf("plev")||Tn(c)),!0):!1}
;function gq(){var a=["ol"];Ep("").info.actionType="embed";a&&uh("TIMING_AFT_KEYS",a);uh("TIMING_ACTION","embed");a=B("TIMING_INFO",{});if(K("web_csi_via_jspb")){var b=new Tg;a=q(Object.entries(a));for(var c=a.next();!c.done;c=a.next()){var d=q(c.value);c=d.next().value;d=d.next().value;switch(c){case "GetBrowse_rid":var e=new Wg;F(e,1,c);F(e,2,String(d));Vg(b,e);break;case "GetGuide_rid":e=new Wg;F(e,1,c);F(e,2,String(d));Vg(b,e);break;case "GetHome_rid":e=new Wg;F(e,1,c);F(e,2,String(d));Vg(b,e);
break;case "GetPlayer_rid":e=new Wg;F(e,1,c);F(e,2,String(d));Vg(b,e);break;case "GetSearch_rid":e=new Wg;F(e,1,c);F(e,2,String(d));Vg(b,e);break;case "GetSettings_rid":e=new Wg;F(e,1,c);F(e,2,String(d));Vg(b,e);break;case "GetTrending_rid":e=new Wg;F(e,1,c);F(e,2,String(d));Vg(b,e);break;case "GetWatchNext_rid":e=new Wg;F(e,1,c);F(e,2,String(d));Vg(b,e);break;case "yt_red":F(b,14,!!d);break;case "yt_ad":F(b,9,!!d)}}hq(b)}else for(b in a)a.hasOwnProperty(b)&&iq(b,a[b]);b={isNavigation:!0,actionType:Fp[vh("TIMING_ACTION")]||
"LATENCY_ACTION_UNKNOWN"};if(a=vh("PREVIOUS_ACTION"))b.previousAction=Fp[a]||"LATENCY_ACTION_UNKNOWN";if(a=B("CLIENT_PROTOCOL"))b.httpProtocol=a;if(a=B("CLIENT_TRANSPORT"))b.transportProtocol=a;(a=ko())&&"UNDEFINED_CSN"!==a&&(b.clientScreenNonce=a);a=jq();if(1===a||-1===a)b.isVisible=!0;a=wp();b.loadType="cold";c=sp();if(d=up())Z("srt",c.responseStart),1!==a.prerender&&Z("_start",d,void 0);a=Cp();0<a&&Z("fpt",a);a=sp();a.isPerformanceNavigationTiming&&kq({performanceNavigationTiming:!0},void 0);Z("nreqs",
a.requestStart,void 0);Z("nress",a.responseStart,void 0);Z("nrese",a.responseEnd,void 0);0<a.redirectEnd-a.redirectStart&&(Z("nrs",a.redirectStart,void 0),Z("nre",a.redirectEnd,void 0));0<a.domainLookupEnd-a.domainLookupStart&&(Z("ndnss",a.domainLookupStart,void 0),Z("ndnse",a.domainLookupEnd,void 0));0<a.connectEnd-a.connectStart&&(Z("ntcps",a.connectStart,void 0),Z("ntcpe",a.connectEnd,void 0));a.secureConnectionStart>=up()&&0<a.connectEnd-a.secureConnectionStart&&(Z("nstcps",a.secureConnectionStart,
void 0),Z("ntcpe",a.connectEnd,void 0));S&&"getEntriesByType"in S&&lq();a=[];if(document.querySelector&&S&&S.getEntriesByName)for(var f in pp)pp.hasOwnProperty(f)&&(c=pp[f],mq(f,c)&&a.push(c));if(0<a.length)for(b.resourceInfo=[],f=q(a),a=f.next();!a.done;a=f.next())b.resourceInfo.push({resourceCache:a.value});kq(b);f=wp();b=yp();if("cold"===f.yt_lt||"cold"===b.loadType){a=rp();c=xp();c=c.gelTicks?c.gelTicks:c.gelTicks={};for(var g in a)g in c||Z(g,a[g]);g={};a=!1;c=q(Object.keys(f));for(d=c.next();!d.done;d=
c.next())d=d.value,(d=Lp(d,f[d]))&&!Bp(yp(void 0),d)&&(fp(b,d),fp(g,d),a=!0);a&&kq(g)}Xa("ytglobal.timingready_",!0);g=vh("TIMING_ACTION");A("ytglobal.timingready_")&&g&&"_start"in rp(void 0)&&qp()&&Ap()}
function iq(a,b,c){null!==b&&(wp(c)[a]=b,(a=Lp(a,b,c))&&kq(a,c))}
function kq(a,b){if(K("web_csi_via_jspb")){for(var c=new Tg,d=Object.keys(a),e=Object.values(a),f=0;f<d.length;f++)switch(d[f]){case "actionType":try{F(c,1,W[e[f]])}catch(l){M(Error("Codegen laipb translator failed to set actionType"))}break;case "clientActionNonce":try{F(c,2,e[f])}catch(l){M(Error("Codegen laipb translator failed to set clientActionNonce"))}break;case "clientScreenNonce":try{F(c,4,e[f])}catch(l){M(Error("Codegen laipb translator failed to set clientScreenNonce"))}break;case "actionVisualElement":try{G(c,
88,e[f])}catch(l){M(Error("Codegen laipb translator failed to set actionVisualElement"))}break;case "loadType":try{F(c,3,e[f])}catch(l){M(Error("Codegen laipb translator failed to set loadType"))}break;case "isFirstInstall":try{F(c,55,e[f])}catch(l){M(Error("Codegen laipb translator failed to set isFirstInstall"))}break;case "networkType":try{F(c,5,Mp[e[f]])}catch(l){M(Error("Codegen laipb translator failed to set networkType"))}break;case "connectionType":try{F(c,26,X[e[f]])}catch(l){M(Error("Codegen laipb translator failed to set connectionType"))}break;
case "detailedConnectionType":try{F(c,27,Y[e[f]])}catch(l){M(Error("Codegen laipb translator failed to set detailedConnectionType"))}break;case "isVisible":try{F(c,6,e[f])}catch(l){M(Error("Codegen laipb translator failed to set isVisible"))}break;case "playerType":try{F(c,7,Np[e[f]])}catch(l){M(Error("Codegen laipb translator failed to set playerType"))}break;case "clientPlaybackNonce":try{F(c,8,e[f])}catch(l){M(Error("Codegen laipb translator failed to set clientPlaybackNonce"))}break;case "adClientPlaybackNonce":try{F(c,
28,e[f])}catch(l){M(Error("Codegen laipb translator failed to set adClientPlaybackNonce"))}break;case "previousCpn":try{F(c,77,e[f])}catch(l){M(Error("Codegen laipb translator failed to set previousCpn"))}break;case "targetCpn":try{F(c,76,e[f])}catch(l){M(Error("Codegen laipb translator failed to set targetCpn"))}break;case "isMonetized":try{F(c,9,e[f])}catch(l){M(Error("Codegen laipb translator failed to set isMonetized"))}break;case "isPrerollAllowed":try{F(c,16,e[f])}catch(l){M(Error("Codegen laipb translator failed to set isPrerollAllowed"))}break;
case "isPrerollShown":try{F(c,17,e[f])}catch(l){M(Error("Codegen laipb translator failed to set isPrerollShown"))}break;case "adType":try{F(c,12,e[f])}catch(l){M(Error("Codegen laipb translator failed to set adType"))}break;case "adTypesAllowed":try{F(c,36,e[f])}catch(l){M(Error("Codegen laipb translator failed to set adTypesAllowed"))}break;case "adNetworks":try{F(c,37,e[f])}catch(l){M(Error("Codegen laipb translator failed to set adNetworks"))}break;case "previousAction":try{F(c,13,W[e[f]])}catch(l){M(Error("Codegen laipb translator failed to set previousAction"))}break;
case "isRedSubscriber":try{F(c,14,e[f])}catch(l){M(Error("Codegen laipb translator failed to set isRedSubscriber"))}break;case "serverTimeMs":try{F(c,15,e[f])}catch(l){M(Error("Codegen laipb translator failed to set serverTimeMs"))}break;case "spinnerInfo":try{G(c,18,e[f])}catch(l){M(Error("Codegen laipb translator failed to set spinnerInfo"))}break;case "videoId":try{c.setVideoId(e[f])}catch(l){M(Error("Codegen laipb translator failed to set videoId"))}break;case "adVideoId":try{F(c,20,e[f])}catch(l){M(Error("Codegen laipb translator failed to set adVideoId"))}break;
case "targetVideoId":try{F(c,78,e[f])}catch(l){M(Error("Codegen laipb translator failed to set targetVideoId"))}break;case "adBreakType":try{F(c,21,Op[e[f]])}catch(l){M(Error("Codegen laipb translator failed to set adBreakType"))}break;case "isNavigation":try{F(c,25,e[f])}catch(l){M(Error("Codegen laipb translator failed to set isNavigation"))}break;case "viewportHeight":try{F(c,29,e[f])}catch(l){M(Error("Codegen laipb translator failed to set viewportHeight"))}break;case "viewportWidth":try{F(c,
30,e[f])}catch(l){M(Error("Codegen laipb translator failed to set viewportWidth"))}break;case "screenHeight":try{F(c,84,e[f])}catch(l){M(Error("Codegen laipb translator failed to set screenHeight"))}break;case "screenWidth":try{F(c,85,e[f])}catch(l){M(Error("Codegen laipb translator failed to set screenWidth"))}break;case "browseId":try{F(c,31,e[f])}catch(l){M(Error("Codegen laipb translator failed to set browseId"))}break;case "isCacheHit":try{F(c,32,e[f])}catch(l){M(Error("Codegen laipb translator failed to set isCacheHit"))}break;
case "httpProtocol":try{F(c,33,e[f])}catch(l){M(Error("Codegen laipb translator failed to set httpProtocol"))}break;case "transportProtocol":try{F(c,34,e[f])}catch(l){M(Error("Codegen laipb translator failed to set transportProtocol"))}break;case "searchQuery":try{F(c,41,e[f])}catch(l){M(Error("Codegen laipb translator failed to set searchQuery"))}break;case "isContinuation":try{F(c,42,e[f])}catch(l){M(Error("Codegen laipb translator failed to set isContinuation"))}break;case "availableProcessors":try{F(c,
43,e[f])}catch(l){M(Error("Codegen laipb translator failed to set availableProcessors"))}break;case "sdk":try{F(c,44,e[f])}catch(l){M(Error("Codegen laipb translator failed to set sdk"))}break;case "isLocalStream":try{F(c,45,e[f])}catch(l){M(Error("Codegen laipb translator failed to set isLocalStream"))}break;case "navigationRequestedSameUrl":try{F(c,64,e[f])}catch(l){M(Error("Codegen laipb translator failed to set navigationRequestedSameUrl"))}break;case "shellStartupDurationMs":try{F(c,70,e[f])}catch(l){M(Error("Codegen laipb translator failed to set shellStartupDurationMs"))}break;
case "appInstallDataAgeMs":try{F(c,73,e[f])}catch(l){M(Error("Codegen laipb translator failed to set appInstallDataAgeMs"))}break;case "latencyActionError":try{F(c,71,Pp[e[f]])}catch(l){M(Error("Codegen laipb translator failed to set latencyActionError"))}break;case "actionStep":try{F(c,79,e[f])}catch(l){M(Error("Codegen laipb translator failed to set actionStep"))}break;case "jsHeapSizeLimit":try{F(c,80,e[f])}catch(l){M(Error("Codegen laipb translator failed to set jsHeapSizeLimit"))}break;case "totalJsHeapSize":try{F(c,
81,e[f])}catch(l){M(Error("Codegen laipb translator failed to set totalJsHeapSize"))}break;case "usedJsHeapSize":try{F(c,82,e[f])}catch(l){M(Error("Codegen laipb translator failed to set usedJsHeapSize"))}break;case "resourceInfo":try{Xc(c,83,Sg,e[f])}catch(l){M(Error("Codegen laipb translator failed to set resourceInfo"))}break;case "sourceVideoDurationMs":try{F(c,90,e[f])}catch(l){M(Error("Codegen laipb translator failed to set sourceVideoDurationMs"))}break;case "playerInfo":try{G(c,22,e[f])}catch(l){M(Error("Codegen laipb translator failed to set playerInfo"))}break;
case "commentInfo":try{G(c,23,e[f])}catch(l){M(Error("Codegen laipb translator failed to set commentInfo"))}break;case "mdxInfo":try{G(c,24,e[f])}catch(l){M(Error("Codegen laipb translator failed to set mdxInfo"))}break;case "watchInfo":try{G(c,35,e[f])}catch(l){M(Error("Codegen laipb translator failed to set watchInfo"))}break;case "adPrebufferedTimeSecs":try{F(c,39,e[f])}catch(l){M(Error("Codegen laipb translator failed to set adPrebufferedTimeSecs"))}break;case "thumbnailLoadInfo":try{G(c,40,e[f])}catch(l){M(Error("Codegen laipb translator failed to set thumbnailLoadInfo"))}break;
case "creatorInfo":try{G(c,46,e[f])}catch(l){M(Error("Codegen laipb translator failed to set creatorInfo"))}break;case "unpluggedInfo":try{G(c,50,e[f])}catch(l){M(Error("Codegen laipb translator failed to set unpluggedInfo"))}break;case "isLivestream":try{F(c,47,e[f])}catch(l){M(Error("Codegen laipb translator failed to set isLivestream"))}break;case "liveStreamMode":try{F(c,91,Qp[e[f]])}catch(l){M(Error("Codegen laipb translator failed to set liveStreamMode"))}break;case "adCpn2":try{F(c,48,e[f])}catch(l){M(Error("Codegen laipb translator failed to set adCpn2"))}break;
case "adDaiDriftMillis":try{F(c,49,e[f])}catch(l){M(Error("Codegen laipb translator failed to set adDaiDriftMillis"))}break;case "videoStreamType":try{F(c,53,Rp[e[f]])}catch(l){M(Error("Codegen laipb translator failed to set videoStreamType"))}break;case "reelInfo":try{G(c,54,e[f])}catch(l){M(Error("Codegen laipb translator failed to set reelInfo"))}break;case "subscriptionsFeedInfo":try{G(c,72,e[f])}catch(l){M(Error("Codegen laipb translator failed to set subscriptionsFeedInfo"))}break;case "playbackRequiresTap":try{F(c,
56,e[f])}catch(l){M(Error("Codegen laipb translator failed to set playbackRequiresTap"))}break;case "requestIds":try{Vg(c,e[f])}catch(l){M(Error("Codegen laipb translator failed to set requestIds"))}break;case "mediaBrowserActionInfo":try{G(c,58,e[f])}catch(l){M(Error("Codegen laipb translator failed to set mediaBrowserActionInfo"))}break;case "performanceNavigationTiming":try{F(c,67,e[f])}catch(l){M(Error("Codegen laipb translator failed to set performanceNavigationTiming"))}break;case "musicLoadActionInfo":try{G(c,
69,e[f])}catch(l){M(Error("Codegen laipb translator failed to set musicLoadActionInfo"))}break;case "transactionType":try{F(c,74,Sp[e[f]])}catch(l){M(Error("Codegen laipb translator failed to set transactionType"))}break;case "shoppingInfo":try{G(c,75,e[f])}catch(l){M(Error("Codegen laipb translator failed to set shoppingInfo"))}break;case "prefetchInfo":try{G(c,86,e[f])}catch(l){M(Error("Codegen laipb translator failed to set prefetchInfo"))}break;case "accelerationSession":try{G(c,87,e[f])}catch(l){M(Error("Codegen laipb translator failed to set accelerationSession"))}break;
case "playerRotationType":try{F(c,101,Tp[e[f]])}catch(l){M(Error("Codegen laipb translator failed to set playerRotationType"))}break;case "webInfo":try{G(c,38,e[f])}catch(l){M(Error("Codegen laipb translator failed to set webInfo"))}break;case "tvInfo":try{G(c,51,e[f])}catch(l){M(Error("Codegen laipb translator failed to set tvInfo"))}break;case "kabukiInfo":try{G(c,52,e[f])}catch(l){M(Error("Codegen laipb translator failed to set kabukiInfo"))}break;case "mwebInfo":try{G(c,59,e[f])}catch(l){M(Error("Codegen laipb translator failed to set mwebInfo"))}break;
case "musicInfo":try{G(c,65,e[f])}catch(l){M(Error("Codegen laipb translator failed to set musicInfo"))}break;case "allowedPreroll":try{F(c,10,e[f])}catch(l){M(Error("Codegen laipb translator failed to set allowedPreroll"))}break;case "shownPreroll":try{F(c,11,e[f])}catch(l){M(Error("Codegen laipb translator failed to set shownPreroll"))}break;case "getHomeRequestId":try{F(c,57,e[f])}catch(l){M(Error("Codegen laipb translator failed to set getHomeRequestId"))}break;case "getSearchRequestId":try{F(c,
60,e[f])}catch(l){M(Error("Codegen laipb translator failed to set getSearchRequestId"))}break;case "getPlayerRequestId":try{F(c,61,e[f])}catch(l){M(Error("Codegen laipb translator failed to set getPlayerRequestId"))}break;case "getWatchNextRequestId":try{F(c,62,e[f])}catch(l){M(Error("Codegen laipb translator failed to set getWatchNextRequestId"))}break;case "getBrowseRequestId":try{F(c,63,e[f])}catch(l){M(Error("Codegen laipb translator failed to set getBrowseRequestId"))}break;case "getLibraryRequestId":try{F(c,
66,e[f])}catch(l){M(Error("Codegen laipb translator failed to set getLibraryRequestId"))}}hq(c,b)}else{var g=Ep(b||"");fp(g.info,a);fp(yp(b),a);var h=zp(b),k=vp(b).cttAuthInfo;eq().info(a,h,k)}}
function hq(a,b){var c=xp(b);c.jspbInfos||(c.jspbInfos=[]);c.jspbInfos.push(a);Ep(b||"").jspbInfo.push(a);c=zp(b);b=vp(b).cttAuthInfo;eq().jspbInfo(a,c,b)}
function Z(a,b,c){if(!b&&"_"!==a[0]){var d=a;S.mark&&(0==d.lastIndexOf("mark_",0)||(d="mark_"+d),c&&(d+=" ("+c+")"),S.mark(d))}Ep(c||"").tick[a]=b||O();d=xp(c);d.gelTicks&&(d.gelTicks[a]=!0);d=rp(c);var e=b||O();d[a]=e;e=zp(c);var f=vp(c).cttAuthInfo;if("_start"===a){var g=eq();fq(g,"baseline_"+e)||(b={timestamp:b,cttAuthInfo:f},K("web_csi_via_jspb")?(f=new Rg,F(f,1,e),e=new $g,Vc(e,6,f),Wp(e,b)):wk("latencyActionBaselined",{clientActionNonce:e},b))}else eq().tick(a,e,b,f);Ap(c);return d[a]}
function nq(){var a=zp(void 0);requestAnimationFrame(function(){setTimeout(function(){a===zp(void 0)&&Z("ol",void 0,void 0)},0)})}
function jq(){var a=document;if("visibilityState"in a)a=a.visibilityState;else{var b=Dh+"VisibilityState";a=b in a?a[b]:void 0}switch(a){case "hidden":return 0;case "visible":return 1;case "prerender":return 2;case "unloaded":return 3;default:return-1}}
function mq(a,b){a=document.querySelector(a);if(!a)return!1;var c="",d=a.nodeName;"SCRIPT"===d?(c=a.src,c||(c=a.getAttribute("data-timing-href"))&&(c=window.location.protocol+c)):"LINK"===d&&(c=a.href);Vb()&&a.setAttribute("nonce",Vb());return c?(a=S.getEntriesByName(c))&&a[0]&&(a=a[0],c=up(),Z("rsf_"+b,c+Math.round(a.fetchStart)),Z("rse_"+b,c+Math.round(a.responseEnd)),void 0!==a.transferSize&&0===a.transferSize)?!0:!1:!1}
function lq(){var a=window.location.protocol,b=S.getEntriesByType("resource");b=gb(b,function(c){return 0===c.name.indexOf(a+"//fonts.gstatic.com/s/")});
(b=ib(b,function(c,d){return d.duration>c.duration?d:c},{duration:0}))&&0<b.startTime&&0<b.responseEnd&&(Z("wffs",tp(b.startTime)),Z("wffe",tp(b.responseEnd)))}
var oq=window;oq.ytcsi&&(oq.ytcsi.info=iq,oq.ytcsi.tick=Z);var pq=["consistency","mss","client_location","entities","store"];function qq(a,b,c,d){this.o=a;this.J=b;this.l=c;this.j=d;this.i=void 0;this.h=new Map;a.Aa||(a.Aa={});a.Aa=Object.assign(Object.assign({},ep),a.Aa)}
function rq(a,b,c,d){if(void 0!==qq.h){if(d=qq.h,a=[a!==d.o,b!==d.J,c!==d.l,!1,!1,void 0!==d.i],a.some(function(e){return e}))throw new N("InnerTubeTransportService is already initialized",a);
}else qq.h=new qq(a,b,c,d)}
function sq(a){var b={signalServiceEndpoint:{signal:"GET_DATASYNC_IDS"}};var c=void 0===c?Jo:c;var d=Uo(b,a.o);if(!d)return Of(new N("Error: No request builder found for command.",b));var e=d.o(b,void 0,c);return e?new Jf(function(f){var g,h,k;return w(function(l){if(1==l.h){h="cors"===(null===(g=e.sa)||void 0===g?void 0:g.mode)?"cors":void 0;if(a.l.xc){var n=e.config,t;n=null===(t=null===n||void 0===n?void 0:n.Ha)||void 0===t?void 0:t.sessionIndex;t=Io({sessionIndex:n});k=Object.assign(Object.assign({},
tq(h)),t);return l.s(2)}return v(l,uq(e.config,h),3)}2!=l.h&&(k=l.i);f(vq(a,e,k));l.h=0})}):Of(new N("Error: Failed to build request for command.",b))}
function vq(a,b,c){var d,e,f,g,h,k,l,n,t,y,u,D,E,L,P,R,V;return w(function(aa){switch(aa.h){case 1:aa.s(2);break;case 3:if((l=aa.i)&&!l.isExpired())return aa.return(Promise.resolve(l.h()));case 2:if((n=null===(d=b.config)||void 0===d?void 0:d.Fo)&&a.h.has(n)&&K("web_memoize_inflight_requests"))return aa.return(a.h.get(n));if(null===(e=null===b||void 0===b?void 0:b.ba)||void 0===e?0:e.context)for(t=q([]),y=t.next();!y.done;y=t.next())u=y.value,u.Do(b.ba.context);if(null===(f=a.i)||void 0===f?0:f.l(b.input,
b.ba))return aa.return(a.i.j(b.input,b.ba));D=JSON.stringify(b.ba);b.sa=Object.assign(Object.assign({},b.sa),{headers:c});E=Object.assign({},b.sa);"POST"===b.sa.method&&(E=Object.assign(Object.assign({},E),{body:D}));(null===(g=b.config)||void 0===g?0:g.ic)&&Z(b.config.ic);L=a.J.fetch(b.input,E,b.config);n&&a.h.set(n,L);return v(aa,L,4);case 4:P=aa.i;n&&a.h.has(n)&&a.h.delete(n);(null===(h=b.config)||void 0===h?0:h.jc)&&Z(b.config.jc);if(P||null===(k=a.i)||void 0===k||!k.h(b.input,b.ba)){aa.s(5);
break}return v(aa,a.i.i(b.input,b.ba),6);case 6:P=aa.i;case 5:if(P&&a.j)for(R=q(pq),y=R.next();!y.done;y=R.next())V=y.value,a.j[V]&&a.j[V].handleResponse(P,b);return aa.return(P)}})}
function uq(a,b){var c,d,e;return w(function(f){if(1==f.h){d=null===(c=null===a||void 0===a?void 0:a.Ha)||void 0===c?void 0:c.sessionIndex;var g=Io({sessionIndex:d});if(!(g instanceof Jf)){var h=new Jf(La);Kf(h,2,g);g=h}return v(f,g,2)}e=f.i;return f.return(Promise.resolve(Object.assign(Object.assign({},tq(b)),e)))})}
function tq(a){var b={"Content-Type":"application/json"};K("enable_web_eom_visitor_data")&&B("EOM_VISITOR_DATA")?b["X-Goog-EOM-Visitor-Id"]=B("EOM_VISITOR_DATA"):B("VISITOR_DATA")&&(b["X-Goog-Visitor-Id"]=B("VISITOR_DATA"));"cors"!==a&&((a=B("INNERTUBE_CONTEXT_CLIENT_NAME"))&&(b["X-Youtube-Client-Name"]=a),(a=B("INNERTUBE_CONTEXT_CLIENT_VERSION"))&&(b["X-Youtube-Client-Version"]=a),(a=B("CHROME_CONNECTED_HEADER"))&&(b["X-Youtube-Chrome-Connected"]=a),K("forward_domain_admin_state_on_embeds")&&(a=
B("DOMAIN_ADMIN_STATE"))&&(b["X-Youtube-Domain-Admin-State"]=a));return b}
;var wq=["share/get_web_player_share_panel"],xq=["feedback"],yq=["notification/modify_channel_preference"],zq=["browse/edit_playlist"],Aq=["subscription/subscribe"],Bq=["subscription/unsubscribe"];function Cq(){}
r(Cq,bp);Cq.prototype.j=function(){return Aq};
Cq.prototype.h=function(a){return a.subscribeEndpoint};
Cq.prototype.i=function(a,b,c){c=void 0===c?{}:c;b.channelIds&&(a.channelIds=b.channelIds);b.siloName&&(a.siloName=b.siloName);b.params&&(a.params=b.params);c.botguardResponse&&(a.botguardResponse=c.botguardResponse);c.feature&&(a.clientFeature=c.feature)};
fa.Object.defineProperties(Cq.prototype,{l:{configurable:!0,enumerable:!0,get:function(){return!0}}});function Dq(){}
r(Dq,bp);Dq.prototype.j=function(){return Bq};
Dq.prototype.h=function(a){return a.unsubscribeEndpoint};
Dq.prototype.i=function(a,b){b.channelIds&&(a.channelIds=b.channelIds);b.siloName&&(a.siloName=b.siloName);b.params&&(a.params=b.params)};
fa.Object.defineProperties(Dq.prototype,{l:{configurable:!0,enumerable:!0,get:function(){return!0}}});function Eq(){}
r(Eq,bp);Eq.prototype.j=function(){return xq};
Eq.prototype.h=function(a){return a.feedbackEndpoint};
Eq.prototype.i=function(a,b,c){a.feedbackTokens=[];b.feedbackToken&&a.feedbackTokens.push(b.feedbackToken);if(b=b.cpn||c.cpn)a.feedbackContext={cpn:b};a.isFeedbackTokenUnencrypted=!!c.is_feedback_token_unencrypted;a.shouldMerge=!1;c.extra_feedback_tokens&&(a.shouldMerge=!0,a.feedbackTokens=a.feedbackTokens.concat(c.extra_feedback_tokens))};
fa.Object.defineProperties(Eq.prototype,{l:{configurable:!0,enumerable:!0,get:function(){return!0}}});function Fq(){}
r(Fq,bp);Fq.prototype.j=function(){return yq};
Fq.prototype.h=function(a){return a.modifyChannelNotificationPreferenceEndpoint};
Fq.prototype.i=function(a,b){b.params&&(a.params=b.params);b.secondaryParams&&(a.secondaryParams=b.secondaryParams)};function Gq(){}
r(Gq,bp);Gq.prototype.j=function(){return zq};
Gq.prototype.h=function(a){return a.playlistEditEndpoint};
Gq.prototype.i=function(a,b){b.actions&&(a.actions=b.actions);b.params&&(a.params=b.params);b.playlistId&&(a.playlistId=b.playlistId)};function Hq(){}
r(Hq,bp);Hq.prototype.j=function(){return wq};
Hq.prototype.h=function(a){return a.webPlayerShareEntityServiceEndpoint};
Hq.prototype.i=function(a,b,c){c=void 0===c?{}:c;b.serializedShareEntity&&(a.serializedSharedEntity=b.serializedShareEntity);c.includeListId&&(a.includeListId=!0)};var Iq=new Wo;function Jq(){}
Jq.prototype.fetch=function(a,b){var c=this,d,e,f;return w(function(g){if(1==g.h){d=new window.Request(a,b);if(K("web_fetch_promise_cleanup_killswitch"))return g.return(Promise.resolve(fetch(d).then(function(h){return c.handleResponse(h)}).catch(function(h){Tn(h)})));
ua(g,3);return v(g,fetch(d),5)}if(3!=g.h)return e=g.i,g.return(c.handleResponse(e));f=wa(g);Tn(f);return g.return(void 0)})};
Jq.prototype.handleResponse=function(a){var b=a.text().then(function(c){return JSON.parse(c.replace(")]}'",""))});
a.redirected||a.ok||(b=b.then(function(c){Tn(new N("Error: API fetch failed",a.status,a.url,c));return Object.assign(Object.assign({},c),{errorMetadata:{status:a.status}})}));
return b};
Jq[Vo]=[new function(a){this.key=a}(Iq)];var Kq;function Lq(a){bm.call(this,1,arguments);this.csn=a}
r(Lq,bm);var km=new cm("screen-created",Lq),Mq=[],Oq=Nq,Pq=0;function Qq(a,b,c,d,e,f,g){function h(){Tn(new N("newScreen() parent element does not have a VE - rootVe",b))}
var k=Oq();f=new co({veType:b,youtubeData:f,jspbYoutubeData:void 0});e={cttAuthInfo:e,X:k};if(K("il_via_jspb")){var l=new Hg;l.Y(k);Ig(l,f.getAsJspb());c&&c.visualElement?(f=new Jg,c.clientScreenNonce&&F(f,2,c.clientScreenNonce),Kg(f,c.visualElement.getAsJspb()),g&&F(f,4,Gg[g]),G(l,5,f)):c&&h();d&&F(l,3,d);aq(l,e,a)}else f={csn:k,pageVe:f.getAsJson()},c&&c.visualElement?(f.implicitGesture={parentCsn:c.clientScreenNonce,gesturedVe:c.visualElement.getAsJson()},g&&(f.implicitGesture.gestureType=g)):
c&&h(),d&&(f.cloneCsn=d),a?Dj("screenCreated",f,a,e):wk("screenCreated",f,e);hm(km,new Lq(k));return k}
function Rq(a,b,c,d){var e=d.filter(function(k){k.csn!==b?(k.csn=b,k=!0):k=!1;return k}),f={cttAuthInfo:mo(b),
X:b};d=q(d);for(var g=d.next();!g.done;g=d.next())g=g.value.getAsJson(),(qb(g)||!g.trackingParams&&!g.veType)&&Tn(Error("Child VE logged with no data"));if(K("il_via_jspb")){var h=new Lg;h.Y(b);Ng(h,c.getAsJspb());hb(e,function(k){k=k.getAsJspb();Xc(h,3,Fg,k)});
"UNDEFINED_CSN"==b?Sq("visualElementAttached",h,f):bq(h,f,a)}else c={csn:b,parentVe:c.getAsJson(),childVes:hb(e,function(k){return k.getAsJson()})},"UNDEFINED_CSN"==b?Sq("visualElementAttached",c,f):a?Dj("visualElementAttached",c,a,f):wk("visualElementAttached",c,f)}
function Nq(){for(var a=Math.random()+"",b=[],c=0,d=0;d<a.length;d++){var e=a.charCodeAt(d);255<e&&(b[c++]=e&255,e>>=8);b[c++]=e}return Bc(b,3)}
function Sq(a,b,c){Mq.push({payloadName:a,payload:b,options:c});Pq||(Pq=lm())}
function mm(a){if(Mq){for(var b=q(Mq),c=b.next();!c.done;c=b.next())if(c=c.value,c.payload)if(K("il_via_jspb"))switch(c.payload.Y(a.csn),c.payloadName){case "screenCreated":aq(c.payload,c.options);break;case "visualElementAttached":bq(c.payload,c.options);break;case "visualElementShown":Xp(c.payload,c.options);break;case "visualElementHidden":Yp(c.payload,c.options);break;case "visualElementGestured":Zp(c.payload,c.options);break;case "visualElementStateChanged":$p(c.payload,c.options);break;default:Tn(new N("flushQueue unable to map payloadName to JSPB setter"))}else c.payload.csn=
a.csn,Dj(c.payloadName,c.payload,null,c.options);Mq.length=0}Pq=0}
;function Tq(){this.i=new Set;this.h=new Set;this.j=new Map}
Tq.prototype.clear=function(){this.i.clear();this.h.clear();this.j.clear()};
Ma(Tq);function Uq(){this.o=[];this.D=[];this.h=[];this.l=[];this.m=[];this.i=new Set;this.u=new Map}
function Vq(a,b,c){c=void 0===c?0:c;b.then(function(d){var e,f;a.i.has(c)&&a.j&&a.j();var g=ko(c),h=io(c);g&&h&&((null===(e=null===d||void 0===d?void 0:d.response)||void 0===e?0:e.trackingParams)&&Rq(a.client,g,h,[eo(d.response.trackingParams)]),(null===(f=null===d||void 0===d?void 0:d.playerResponse)||void 0===f?0:f.trackingParams)&&Rq(a.client,g,h,[eo(d.playerResponse.trackingParams)]))})}
function Wq(a,b,c,d){d=void 0===d?0:d;if(a.i.has(d))a.o.push([b,c]);else{var e=ko(d);c=c||io(d);e&&c&&Rq(a.client,e,c,[b])}}
Uq.prototype.clickCommand=function(a,b,c){a:{var d=a.clickTrackingParams;c=void 0===c?0:c;c=void 0===c?0:c;if(d){if(K("web_ignore_no_ve_clicks")&&(a=Hn(atob(d.replace(/-/g,"+").replace(/_/g,"/"))),!a||0===a)){b=!1;break a}if(c=ko(c)){a=this.client;var e=eo(d);var f="INTERACTION_LOGGING_GESTURE_TYPE_GENERIC_CLICK";d={cttAuthInfo:mo(c),X:c};if(K("il_via_jspb")){var g=new Og;g.Y(c);e=e.getAsJspb();G(g,2,e);F(g,4,Gg[f]);b&&G(g,3,void 0);"UNDEFINED_CSN"==c?Sq("visualElementGestured",g,d):Zp(g,d,a)}else f=
{csn:c,ve:e.getAsJson(),gestureType:f},b&&(f.clientData=b),"UNDEFINED_CSN"==c?Sq("visualElementGestured",f,d):a?Dj("visualElementGestured",f,a,d):wk("visualElementGestured",f,d);b=!0}else b=!1}else b=!1}return b};
function Xq(a,b,c){c=void 0===c?{}:c;a.i.add(c.layer||0);a.j=function(){Yq(a,b,c);var f=io(c.layer);if(f){for(var g=q(a.o),h=g.next();!h.done;h=g.next())h=h.value,Wq(a,h[0],h[1]||f,c.layer);f=q(a.D);for(g=f.next();!g.done;g=f.next()){var k=g.value;g=void 0;g=void 0===g?0:g;h=ko(g);var l=k[0]||io(g);if(h&&l){g=a.client;var n=k[1];k={cttAuthInfo:mo(h),X:h};K("il_via_jspb")?(n=new Qg,n.Y(h),l=l.getAsJspb(),G(n,2,l),"UNDEFINED_CSN"==h?Sq("visualElementStateChanged",n,k):$p(n,k,g)):(l={csn:h,ve:l.getAsJson(),
clientData:n},"UNDEFINED_CSN"==h?Sq("visualElementStateChanged",l,k):g?Dj("visualElementStateChanged",l,g,k):wk("visualElementStateChanged",l,k))}}}};
ko(c.layer)||a.j();if(c.sb)for(var d=q(c.sb),e=d.next();!e.done;e=d.next())Vq(a,e.value,c.layer);else Sn(Error("Delayed screen needs a data promise."))}
function Yq(a,b,c){c=void 0===c?{}:c;c.layer||(c.layer=0);var d=void 0!==c.dc?c.dc:c.layer;var e=ko(d);d=io(d);var f;d&&(void 0!==c.parentCsn?f={clientScreenNonce:c.parentCsn,visualElement:d}:e&&"UNDEFINED_CSN"!==e&&(f={clientScreenNonce:e,visualElement:d}));var g,h=B("EVENT_ID");"UNDEFINED_CSN"===e&&h&&(g={servletData:{serializedServletEventId:h}});try{var k=Qq(a.client,b,f,c.rb,c.cttAuthInfo,g,c.Ao)}catch(l){Un(l,{Go:b,rootVe:d,parentVisualElement:void 0,xo:e,Co:f,rb:c.rb});Sn(l);return}no(k,b,
c.layer,c.cttAuthInfo);if(b=e&&"UNDEFINED_CSN"!==e&&d){a:{b=q(Object.values(bo));for(f=b.next();!f.done;f=b.next())if(ko(f.value)==e){b=!0;break a}b=!1}b=!b}b&&(b=a.client,g=!0,h=(g=void 0===g?!1:g)?16:8,f={cttAuthInfo:mo(e),X:e,ub:g},K("il_via_jspb")?(h=new Pg,h.Y(e),d=d.getAsJspb(),G(h,2,d),F(h,4,g?16:8),"UNDEFINED_CSN"==e?Sq("visualElementHidden",h,f):Yp(h,f,b)):(d={csn:e,ve:d.getAsJson(),eventType:h},"UNDEFINED_CSN"==e?Sq("visualElementHidden",d,f):b?Dj("visualElementHidden",d,b,f):wk("visualElementHidden",
d,f)));a.h[a.h.length-1]&&!a.h[a.h.length-1].csn&&(a.h[a.h.length-1].csn=k||"");kq({clientScreenNonce:k});Tq.getInstance().clear();d=io(c.layer);e&&"UNDEFINED_CSN"!==e&&d&&(K("web_mark_root_visible")||K("music_web_mark_root_visible"))&&(e=k,k={cttAuthInfo:mo(e),X:e},K("il_via_jspb")?(b=new Pg,b.Y(e),f=d.getAsJspb(),G(b,2,f),F(b,4,1),"UNDEFINED_CSN"==e?Sq("visualElementShown",b,k):Xp(b,k,void 0)):(b={csn:e,ve:d.getAsJson(),eventType:1},"UNDEFINED_CSN"==e?Sq("visualElementShown",b,k):wk("visualElementShown",
b,k)));a.i.delete(c.layer||0);a.j=void 0;e=q(a.u);for(k=e.next();!k.done;k=e.next())b=q(k.value),k=b.next().value,b=b.next().value,b.has(c.layer)&&d&&Wq(a,k,d,c.layer);for(c=0;c<a.l.length;c++){e=a.l[c];try{e()}catch(l){Sn(l)}}for(c=a.l.length=0;c<a.m.length;c++){e=a.m[c];try{e()}catch(l){Sn(l)}}}
;function Zq(){var a,b;return w(function(c){if(1==c.h)return a=qq.h,a?v(c,sq(a),2):(Tn(Error("InnertubeTransportService unavailable in fetchDatasyncIds")),c.return(void 0));if(b=c.i)return b.errorMetadata?(Sn(Error("Datasync IDs fetch responded with "+b.errorMetadata.code+": "+b.error)),c.return(void 0)):c.return(b.yo);Tn(Error("Network request to get Datasync IDs failed."));return c.return(void 0)})}
;var $q=x.caches,ar;function br(a){var b=a.indexOf(":");return-1===b?{Cb:a}:{Cb:a.substring(0,b),datasyncId:a.substring(b+1)}}
function cr(){return w(function(a){if(void 0!==ar)return a.return(ar);ar=new Promise(function(b){var c;return w(function(d){switch(d.h){case 1:return ua(d,2),v(d,$q.open("test-only"),4);case 4:return v(d,$q.delete("test-only"),5);case 5:va(d,3);break;case 2:if(c=wa(d),c instanceof Error&&"SecurityError"===c.name)return b(!1),d.return();case 3:b("caches"in window),d.h=0}})});
return a.return(ar)})}
function dr(a){var b,c,d,e,f,g,h;w(function(k){if(1==k.h)return v(k,cr(),2);if(3!=k.h){if(!k.i)return k.return(!1);b=[];return v(k,$q.keys(),3)}c=k.i;d=q(c);for(e=d.next();!e.done;e=d.next())f=e.value,g=br(f),h=g.datasyncId,!h||a.includes(h)||b.push($q.delete(f));return k.return(Promise.all(b).then(function(l){return l.some(function(n){return n})}))})}
function er(){var a,b,c,d,e,f,g;return w(function(h){if(1==h.h)return v(h,cr(),2);if(3!=h.h){if(!h.i)return h.return(!1);a=Ak("cache contains other");return v(h,$q.keys(),3)}b=h.i;c=q(b);for(d=c.next();!d.done;d=c.next())if(e=d.value,f=br(e),(g=f.datasyncId)&&g!==a)return h.return(!0);return h.return(!1)})}
;function fr(){try{return!!self.localStorage}catch(a){return!1}}
;function gr(a){a=a.match(/(.*)::.*::.*/);if(null!==a)return a[1]}
function hr(a){if(fr()){var b=Object.keys(window.localStorage);b=q(b);for(var c=b.next();!c.done;c=b.next()){c=c.value;var d=gr(c);void 0===d||a.includes(d)||self.localStorage.removeItem(c)}}}
function ir(){if(!fr())return!1;var a=Ak(),b=Object.keys(window.localStorage);b=q(b);for(var c=b.next();!c.done;c=b.next())if(c=gr(c.value),void 0!==c&&c!==a)return!0;return!1}
;function jr(){Zq().then(function(a){a&&(Kl(a),dr(a),hr(a))})}
function kr(){var a=new Wm;ai.N(function(){var b,c,d,e;return w(function(f){switch(f.h){case 1:if(K("ytidb_clear_optimizations_killswitch")){f.s(2);break}b=Ak("clear");if(b.startsWith("V")){var g=[b];Kl(g);dr(g);hr(g);return f.return()}c=ir();return v(f,er(),3);case 3:return d=f.i,v(f,Ll(),4);case 4:if(e=f.i,!c&&!d&&!e)return f.return();case 2:a.H()?jr():a.i.add("publicytnetworkstatus-online",jr,!0,void 0,void 0),f.h=0}})})}
;function lr(a){a&&(a.dataset?a.dataset[mr("loaded")]="true":a.setAttribute("data-loaded","true"))}
function nr(a,b){return a?a.dataset?a.dataset[mr(b)]:a.getAttribute("data-"+b):null}
var or={};function mr(a){return or[a]||(or[a]=String(a).replace(/\-([a-z])/g,function(b,c){return c.toUpperCase()}))}
;var pr=/\.vflset|-vfl[a-zA-Z0-9_+=-]+/,qr=/-[a-zA-Z]{2,3}_[a-zA-Z]{2,3}(?=(\/|$))/;function rr(a,b,c){c=void 0===c?null:c;if(window.spf&&spf.script){c="";if(a){var d=a.indexOf("jsbin/"),e=a.lastIndexOf(".js"),f=d+6;-1<d&&-1<e&&e>f&&(c=a.substring(f,e),c=c.replace(pr,""),c=c.replace(qr,""),c=c.replace("debug-",""),c=c.replace("tracing-",""))}spf.script.load(a,c,b)}else sr(a,b,c)}
function sr(a,b,c){c=void 0===c?null:c;var d=tr(a),e=document.getElementById(d),f=e&&nr(e,"loaded"),g=e&&!f;f?b&&b():(b&&(f=Pi(d,b),b=""+Qa(b),ur[b]=f),g||(e=vr(a,d,function(){nr(e,"loaded")||(lr(e),Si(d),Rh(Wa(Ti,d),0))},c)))}
function vr(a,b,c,d){d=void 0===d?null:d;var e=Ad("SCRIPT");e.id=b;e.onload=function(){c&&setTimeout(c,0)};
e.onreadystatechange=function(){switch(e.readyState){case "loaded":case "complete":e.onload()}};
d&&e.setAttribute("nonce",d);wd(e,Af(a));a=document.getElementsByTagName("head")[0]||document.body;a.insertBefore(e,a.firstChild);return e}
function wr(a){a=tr(a);var b=document.getElementById(a);b&&(Ti(a),b.parentNode.removeChild(b))}
function xr(a,b){a&&b&&(a=""+Qa(b),(a=ur[a])&&Ri(a))}
function tr(a){var b=document.createElement("a");Sb(b,a);a=b.href.replace(/^[a-zA-Z]+:\/\//,"//");return"js-"+Xb(a)}
var ur={};var yr=[],zr=!1;function Ar(){if(!K("disable_biscotti_fetch_for_ad_blocker_detection")&&!K("disable_biscotti_fetch_entirely_for_all_web_clients")&&yo()&&"1"!=sb()){var a=function(){zr=!0;"google_ad_status"in window?uh("DCLKSTAT",1):uh("DCLKSTAT",2)};
try{rr("//static.doubleclick.net/instream/ad_status.js",a)}catch(b){}yr.push(ai.N(function(){if(!(zr||"google_ad_status"in window)){try{xr("//static.doubleclick.net/instream/ad_status.js",a)}catch(b){}zr=!0;uh("DCLKSTAT",3)}},5E3))}}
function Br(){var a=Number(B("DCLKSTAT",0));return isNaN(a)?0:a}
;function Cr(){this.state=1;this.h=null}
m=Cr.prototype;
m.initialize=function(a,b,c){var d,e;if(a.program){var f=null!==(d=a.interpreterScript)&&void 0!==d?d:null,g=null!==(e=a.interpreterUrl)&&void 0!==e?e:null;if(a.interpreterSafeScript){f=a.interpreterSafeScript;Cb("From proto message. b/166824318");f=f.privateDoNotAccessOrElseSafeScriptWrappedValue||"";var h=zb();f=h?h.createScript(f):f;f=(new Eb(f)).toString()}a.interpreterSafeUrl&&(g=a.interpreterSafeUrl,Cb("From proto message. b/166824318"),g=Ib(g.privateDoNotAccessOrElseTrustedResourceUrlWrappedValue||"").toString());
Dr(this,f,g,a.program,b,c)}else Tn(Error("Cannot initialize botguard without program"))};
function Dr(a,b,c,d,e,f){var g=void 0===g?"trayride":g;c?(a.state=2,rr(c,function(){window[g]?Er(a,d,g,e):(a.state=3,wr(c),Tn(new N("Unable to load Botguard","from "+c)))},f)):b?(f=Ad("SCRIPT"),f.textContent=b,f.nonce=Vb(),document.head.appendChild(f),document.head.removeChild(f),window[g]?Er(a,d,g,e):(a.state=4,Tn(new N("Unable to load Botguard from JS")))):Tn(new N("Unable to load VM; no url or JS provided"))}
m.isInitialized=function(){return!!this.h};
m.getState=function(){return this.state};
function Er(a,b,c,d){a.state=5;try{var e=new pd({program:b,globalName:c});e.uc.then(function(){a.state=6;d&&d(b)});
Fr(a,e)}catch(f){a.state=7,f instanceof Error&&Tn(f)}}
m.invoke=function(a){a=void 0===a?{}:a;if(this.h){var b=this.h;a={qb:a};if(b.i)throw Error("Already disposed");b=b.l([a.qb,a.wc])}else b=null;return b};
m.dispose=function(){Fr(this,null);this.state=8};
function Fr(a,b){be(a.h);a.h=b}
;var Gr=new Cr;function Hr(){return Gr.isInitialized()}
function Ir(a){a=void 0===a?{}:a;return Gr.invoke(a)}
;function Jr(a){var b=this;var c=void 0===c?0:c;var d=void 0===d?$h():d;this.l=c;this.j=d;this.i=new od;this.h=a;a={};c=q(this.h.entries());for(d=c.next();!d.done;a={ta:a.ta,Ca:a.Ca},d=c.next()){var e=q(d.value);d=e.next().value;e=e.next().value;a.Ca=d;a.ta=e;d=function(f){return function(){f.ta.fb();b.h[f.Ca].Sa=!0;b.h.every(function(g){return!0===g.Sa})&&b.i.resolve()}}(a);
e=Wh(d,Kr(this,a.ta));this.h[a.Ca]=Object.assign(Object.assign({},a.ta),{fb:d,Na:e})}}
function Lr(a){var b=Array.from(a.h.keys()).sort(function(d,e){return Kr(a,a.h[e])-Kr(a,a.h[d])});
b=q(b);for(var c=b.next();!c.done;c=b.next())c=a.h[c.value],void 0===c.Na||c.Sa||(a.j.U(c.Na),Wh(c.fb,10))}
Jr.prototype.cancel=function(){for(var a=q(this.h),b=a.next();!b.done;b=a.next())b=b.value,void 0===b.Na||b.Sa||this.j.U(b.Na),b.Sa=!0;this.i.resolve()};
function Kr(a,b){var c;return null!==(c=b.priority)&&void 0!==c?c:a.l}
;function Mr(a){this.state=a;this.plugins=[];this.m=void 0}
Mr.prototype.install=function(){this.plugins.push.apply(this.plugins,ia(Fa.apply(0,arguments)))};
Mr.prototype.transition=function(a,b){var c=this,d=this.D.find(function(f){return f.from===c.state&&f.B===a});
if(d){this.j&&(Lr(this.j),this.j=void 0);this.state=a;d=d.action.bind(this);var e=this.plugins.filter(function(f){return f[a]}).map(function(f){return f[a]});
d(Nr(this,e,this.m),b)}else throw Error("no transition specified from "+this.state+" to "+a);};
function Nr(a,b,c){return function(){var d=Fa.apply(0,arguments),e=b.filter(function(k){var l;return 10===(null!==(l=null!==c&&void 0!==c?c:k.priority)&&void 0!==l?l:0)}),f=b.filter(function(k){var l;
return 10!==(null!==(l=null!==c&&void 0!==c?c:k.priority)&&void 0!==l?l:0)});
$h();var g={};e=q(e);for(var h=e.next();!h.done;g={Da:g.Da},h=e.next())g.Da=h.value,Yh(function(k){return function(){k.Da.la.apply(k.Da,ia(d))}}(g));
f=f.map(function(k){var l;return{fb:function(){k.la.apply(k,ia(d))},
priority:null!==(l=null!==c&&void 0!==c?c:k.priority)&&void 0!==l?l:0}});
f.length&&(a.j=new Jr(f))}}
fa.Object.defineProperties(Mr.prototype,{currentState:{configurable:!0,enumerable:!0,get:function(){return this.state}}});function Or(a){Mr.call(this,void 0===a?"document_active":a);var b=this;this.m=10;this.h=new Map;this.D=[{from:"document_active",B:"document_disposed_preventable",action:this.u},{from:"document_active",B:"document_disposed",action:this.l},{from:"document_disposed_preventable",B:"document_disposed",action:this.l},{from:"document_disposed_preventable",B:"flush_logs",action:this.o},{from:"document_disposed_preventable",B:"document_active",action:this.i},{from:"document_disposed",B:"flush_logs",action:this.o},
{from:"document_disposed",B:"document_active",action:this.i},{from:"document_disposed",B:"document_disposed",action:function(){}},
{from:"flush_logs",B:"document_active",action:this.i}];window.addEventListener("pagehide",function(c){b.transition("document_disposed",c)});
window.addEventListener("beforeunload",function(c){b.transition("document_disposed_preventable",c)})}
r(Or,Mr);Or.prototype.u=function(a,b){if(!this.h.get("document_disposed_preventable")&&(a(b),(null===b||void 0===b?0:b.defaultPrevented)||(null===b||void 0===b?0:b.returnValue))){b.returnValue||(b.returnValue=!0);b.defaultPrevented||b.preventDefault();this.h=new Map;this.transition("document_active");return}this.h.set("document_disposed_preventable",!0);this.h.get("document_disposed")?this.transition("flush_logs"):this.transition("document_disposed")};
Or.prototype.l=function(a,b){this.h.get("document_disposed")?this.transition("document_active"):(a(b),this.h.set("document_disposed",!0),this.transition("flush_logs"))};
Or.prototype.o=function(a,b){a(b);this.transition("document_active")};
Or.prototype.i=function(){this.h=new Map};function Pr(a){Mr.call(this,void 0===a?"document_visibility_unknown":a);var b=this;this.D=[{from:"document_visibility_unknown",B:"document_visible",action:this.i},{from:"document_visibility_unknown",B:"document_hidden",action:this.h},{from:"document_visibility_unknown",B:"document_foregrounded",action:this.o},{from:"document_visibility_unknown",B:"document_backgrounded",action:this.l},{from:"document_visible",B:"document_hidden",action:this.h},{from:"document_visible",B:"document_foregrounded",action:this.o},
{from:"document_visible",B:"document_visible",action:this.i},{from:"document_foregrounded",B:"document_visible",action:this.i},{from:"document_foregrounded",B:"document_hidden",action:this.h},{from:"document_foregrounded",B:"document_foregrounded",action:this.o},{from:"document_hidden",B:"document_visible",action:this.i},{from:"document_hidden",B:"document_backgrounded",action:this.l},{from:"document_hidden",B:"document_hidden",action:this.h},{from:"document_backgrounded",B:"document_hidden",action:this.h},
{from:"document_backgrounded",B:"document_backgrounded",action:this.l},{from:"document_backgrounded",B:"document_visible",action:this.i}];document.addEventListener("visibilitychange",function(c){"visible"===document.visibilityState?b.transition("document_visible",c):b.transition("document_hidden",c)});
K("visibility_lifecycles_dynamic_backgrounding")&&(window.addEventListener("blur",function(c){b.transition("document_backgrounded",c)}),window.addEventListener("focus",function(c){b.transition("document_foregrounded",c)}))}
r(Pr,Mr);Pr.prototype.i=function(a,b){a(b);K("visibility_lifecycles_dynamic_backgrounding")&&this.transition("document_foregrounded")};
Pr.prototype.h=function(a,b){a(b);K("visibility_lifecycles_dynamic_backgrounding")&&this.transition("document_backgrounded")};
Pr.prototype.l=function(a,b){a(b)};
Pr.prototype.o=function(a,b){a(b)};function Qr(){this.h=new Or;this.i=new Pr}
Qr.prototype.install=function(){var a=Fa.apply(0,arguments);this.h.install.apply(this.h,ia(a));this.i.install.apply(this.i,ia(a))};function Rr(){Qr.call(this);var a={};this.install((a.document_disposed={la:this.j},a));a={};this.install((a.flush_logs={la:this.l},a))}
var Sr;r(Rr,Qr);Rr.prototype.l=function(){wk("finalPayload",{csn:ko()})};
Rr.prototype.j=function(){Wn(Xn)};function Tr(){}
Tr.getInstance=function(){var a=A("ytglobal.storage_");a||(a=new Tr,z("ytglobal.storage_",a,void 0));return a};
Tr.prototype.estimate=function(){var a,b,c;return w(function(d){c=navigator;return(null===(a=c.storage)||void 0===a?0:a.estimate)?d.return(c.storage.estimate()):(null===(b=c.webkitTemporaryStorage)||void 0===b?0:b.queryUsageAndQuota)?d.return(Ur()):d.return()})};
function Ur(){var a=navigator;return new Promise(function(b,c){var d;null!==(d=a.webkitTemporaryStorage)&&void 0!==d&&d.queryUsageAndQuota?a.webkitTemporaryStorage.queryUsageAndQuota(function(e,f){b({usage:e,quota:f})},function(e){c(e)}):c(Error("webkitTemporaryStorage is not supported."))})}
z("ytglobal.storageClass_",Tr,void 0);function uk(a,b){var c=this;this.handleError=a;this.h=b;this.i=!1;void 0===self.document||self.addEventListener("beforeunload",function(){c.i=!0});
this.j=Math.random()<=xh("ytidb_transaction_ended_event_rate_limit",.02)}
uk.prototype.logEvent=function(a,b){switch(a){case "IDB_DATA_CORRUPTED":K("idb_data_corrupted_killswitch")||this.h("idbDataCorrupted",b);break;case "IDB_UNEXPECTEDLY_CLOSED":this.h("idbUnexpectedlyClosed",b);break;case "IS_SUPPORTED_COMPLETED":K("idb_is_supported_completed_killswitch")||this.h("idbIsSupportedCompleted",b);break;case "QUOTA_EXCEEDED":Vr(this,b);break;case "TRANSACTION_ENDED":this.j&&this.h("idbTransactionEnded",b);break;case "TRANSACTION_UNEXPECTEDLY_ABORTED":a=Object.assign(Object.assign({},
b),{hasWindowUnloaded:this.i}),this.h("idbTransactionAborted",a)}};
function Vr(a,b){Tr.getInstance().estimate().then(function(c){c=Object.assign(Object.assign({},b),{isSw:void 0===self.document,isIframe:self!==self.top,deviceStorageUsageMbytes:Wr(null===c||void 0===c?void 0:c.usage),deviceStorageQuotaMbytes:Wr(null===c||void 0===c?void 0:c.quota)});a.h("idbQuotaExceeded",c)})}
function Wr(a){return"undefined"===typeof a?"-1":String(Math.ceil(a/1048576))}
;var Xr=window;
function Yr(){var a=Xr.uaChPolyfill.state;if(0===a.type)wk("clientHintsPolyfillEvent",{clientHintsSupported:!1});else{var b=navigator.userAgent,c=void 0!==a.syntheticUa&&a.syntheticUa===b,d={clientHintsSupported:!0,uaAccessedBeforePolyfill:a.didAccessUaBeforePolyfillAvailable,syntheticUaMatches:c};a.didAccessUaBeforePolyfillAvailable&&(d.uaAccessBeforePolyfillCount=a.uaAccessBeforePolyfillCount,a.firstAccessUaError&&(d.firstUaAccessStack=String(a.firstAccessUaError.stack).replace(/\n/g,""),Sn(a.firstAccessUaError)),
d.polyfillAvailabilityDelayMs=a.polyfillAvailabilityDelay);wk("clientHintsPolyfillEvent",d);c||(b={syntheticUa:a.syntheticUa,ua:b},b.brand=a.data.brands.map(function(e){return'"'+e.brand+'"; v="'+e.version+'"'}),b.mobileness=a.data.mobile,a=a.data.values,a.architecture&&(b.platformArchitecture=a.architecture),a.model&&(b.model=a.model),a.platform&&(b.platformBrand=a.platform),a.platformVersion&&(b.platformVersion=a.platformVersion),a.uaFullVersion&&(b.fullVersion=a.uaFullVersion),wk("clientHintsPolyfillDiagnostics",
b))}}
var Zr=!1;function $r(){var a;1===(null===(a=Xr.uaChPolyfill)||void 0===a?void 0:a.state.type)?Zr||(Xr.uaChPolyfill.onReady=$r,Zr=!0):Xr.uaChPolyfill&&Yr()}
;function as(a,b,c){I.call(this);var d=this;c=c||B("POST_MESSAGE_ORIGIN",void 0)||window.document.location.protocol+"//"+window.document.location.hostname;this.j=b||null;this.L="*";this.l=c;this.sessionId=null;this.channel="widget";this.M=!!a;this.A=function(e){a:if(!("*"!=d.l&&e.origin!=d.l||d.j&&e.source!=d.j||"string"!==typeof e.data)){try{var f=JSON.parse(e.data)}catch(g){break a}if(!(null==f||d.M&&(d.sessionId&&d.sessionId!=f.id||d.channel&&d.channel!=f.channel))&&f)switch(f.event){case "listening":"null"!=
e.origin&&(d.l=d.L=e.origin);d.j=e.source;d.sessionId=f.id;d.i&&(d.i(),d.i=null);break;case "command":d.m&&(!d.u||0<=eb(d.u,f.func))&&d.m(f.func,f.args,e.origin)}}};
this.u=this.i=this.m=null;window.addEventListener("message",this.A)}
r(as,I);as.prototype.sendMessage=function(a,b){if(b=b||this.j){this.sessionId&&(a.id=this.sessionId);this.channel&&(a.channel=this.channel);try{var c=JSON.stringify(a);b.postMessage(c,this.L)}catch(d){Hh(d)}}};
as.prototype.I=function(){window.removeEventListener("message",this.A);I.prototype.I.call(this)};function bs(){this.i=[];this.isReady=!1;this.j={};var a=this.h=new as(!!B("WIDGET_ID_ENFORCE")),b=this.fc.bind(this);a.m=b;a.u=null;this.h.channel="widget";if(a=B("WIDGET_ID"))this.h.sessionId=a}
m=bs.prototype;m.fc=function(a,b,c){"addEventListener"===a&&b?(a=b[0],this.j[a]||"onReady"===a||(this.addEventListener(a,cs(this,a)),this.j[a]=!0)):this.lb(a,b,c)};
m.lb=function(){};
function cs(a,b){return function(c){return a.sendMessage(b,c)}}
m.addEventListener=function(){};
m.Ub=function(){this.isReady=!0;this.sendMessage("initialDelivery",this.Ya());this.sendMessage("onReady");fb(this.i,this.Ib,this);this.i=[]};
m.Ya=function(){return null};
function ds(a,b){a.sendMessage("infoDelivery",b)}
m.Ib=function(a){this.isReady?this.h.sendMessage(a):this.i.push(a)};
m.sendMessage=function(a,b){this.Ib({event:a,info:void 0===b?null:b})};
m.dispose=function(){this.h=null};function es(a){return(0===a.search("cue")||0===a.search("load"))&&"loadModule"!==a}
function fs(a,b,c){if("string"===typeof a)return{videoId:a,startSeconds:b,suggestedQuality:c};b=["endSeconds","startSeconds","mediaContentUrl","suggestedQuality","videoId"];c={};for(var d=0;d<b.length;d++){var e=b[d];a[e]&&(c[e]=a[e])}return c}
function gs(a,b,c,d){if(Pa(a)&&!Array.isArray(a)){b="playlist list listType index startSeconds suggestedQuality".split(" ");c={};for(d=0;d<b.length;d++){var e=b[d];a[e]&&(c[e]=a[e])}return c}b={index:b,startSeconds:c,suggestedQuality:d};"string"===typeof a&&16===a.length?b.list="PL"+a:b.playlist=a;return b}
;function hs(a){bs.call(this);this.listeners=[];this.api=a;this.addEventListener("onReady",this.onReady.bind(this));this.addEventListener("onVideoProgress",this.qc.bind(this));this.addEventListener("onVolumeChange",this.sc.bind(this));this.addEventListener("onApiChange",this.kc.bind(this));this.addEventListener("onPlaybackQualityChange",this.nc.bind(this));this.addEventListener("onPlaybackRateChange",this.oc.bind(this));this.addEventListener("onStateChange",this.pc.bind(this));this.addEventListener("onWebglSettingsChanged",
this.tc.bind(this))}
r(hs,bs);m=hs.prototype;
m.lb=function(a,b,c){if(this.api.isExternalMethodAvailable(a,c)){b=b||[];if(0<b.length&&es(a)){var d=b;if(Pa(d[0])&&!Array.isArray(d[0]))var e=d[0];else switch(e={},a){case "loadVideoById":case "cueVideoById":e=fs(d[0],void 0!==d[1]?Number(d[1]):void 0,d[2]);break;case "loadVideoByUrl":case "cueVideoByUrl":e=d[0];"string"===typeof e&&(e={mediaContentUrl:e,startSeconds:void 0!==d[1]?Number(d[1]):void 0,suggestedQuality:d[2]});b:{if((d=e.mediaContentUrl)&&(d=/\/([ve]|embed)\/([^#?]+)/.exec(d))&&d[2]){d=
d[2];break b}d=null}e.videoId=d;e=fs(e);break;case "loadPlaylist":case "cuePlaylist":e=gs(d[0],d[1],d[2],d[3])}b.length=1;b[0]=e}this.api.handleExternalCall(a,b,c);es(a)&&ds(this,this.Ya())}};
m.onReady=function(){var a=this.Ub.bind(this);this.h.i=a};
m.addEventListener=function(a,b){this.listeners.push({eventType:a,listener:b});this.api.addEventListener(a,b)};
m.Ya=function(){if(!this.api)return null;var a=this.api.getApiInterface();kb(a,"getVideoData");for(var b={apiInterface:a},c=0,d=a.length;c<d;c++){var e=a[c];if(0===e.search("get")||0===e.search("is")){var f=0;0===e.search("get")?f=3:0===e.search("is")&&(f=2);f=e.charAt(f).toLowerCase()+e.substr(f+1);try{var g=this.api[e]();b[f]=g}catch(h){}}}b.videoData=this.api.getVideoData();b.currentTimeLastUpdated_=Date.now()/1E3;return b};
m.pc=function(a){a={playerState:a,currentTime:this.api.getCurrentTime(),duration:this.api.getDuration(),videoData:this.api.getVideoData(),videoStartBytes:0,videoBytesTotal:this.api.getVideoBytesTotal(),videoLoadedFraction:this.api.getVideoLoadedFraction(),playbackQuality:this.api.getPlaybackQuality(),availableQualityLevels:this.api.getAvailableQualityLevels(),currentTimeLastUpdated_:Date.now()/1E3,playbackRate:this.api.getPlaybackRate(),mediaReferenceTime:this.api.getMediaReferenceTime()};this.api.getVideoUrl&&
(a.videoUrl=this.api.getVideoUrl());this.api.getVideoContentRect&&(a.videoContentRect=this.api.getVideoContentRect());this.api.getProgressState&&(a.progressState=this.api.getProgressState());this.api.getPlaylist&&(a.playlist=this.api.getPlaylist());this.api.getPlaylistIndex&&(a.playlistIndex=this.api.getPlaylistIndex());this.api.getStoryboardFormat&&(a.storyboardFormat=this.api.getStoryboardFormat());ds(this,a)};
m.nc=function(a){ds(this,{playbackQuality:a})};
m.oc=function(a){ds(this,{playbackRate:a})};
m.kc=function(){for(var a=this.api.getOptions(),b={namespaces:a},c=0,d=a.length;c<d;c++){var e=a[c],f=this.api.getOptions(e);b[e]={options:f};for(var g=0,h=f.length;g<h;g++){var k=f[g],l=this.api.getOption(e,k);b[e][k]=l}}this.sendMessage("apiInfoDelivery",b)};
m.sc=function(){ds(this,{muted:this.api.isMuted(),volume:this.api.getVolume()})};
m.qc=function(a){a={currentTime:a,videoBytesLoaded:this.api.getVideoBytesLoaded(),videoLoadedFraction:this.api.getVideoLoadedFraction(),currentTimeLastUpdated_:Date.now()/1E3,playbackRate:this.api.getPlaybackRate(),mediaReferenceTime:this.api.getMediaReferenceTime()};this.api.getProgressState&&(a.progressState=this.api.getProgressState());ds(this,a)};
m.tc=function(){var a={sphericalProperties:this.api.getSphericalProperties()};ds(this,a)};
m.dispose=function(){bs.prototype.dispose.call(this);for(var a=0;a<this.listeners.length;a++){var b=this.listeners[a];this.api.removeEventListener(b.eventType,b.listener)}this.listeners=[]};function is(a){I.call(this);this.i={};this.started=!1;this.connection=a;this.connection.subscribe("command",this.Eb,this)}
r(is,I);m=is.prototype;m.start=function(){this.started||this.h()||(this.started=!0,this.connection.ma("RECEIVING"))};
m.ma=function(a,b){this.started&&!this.h()&&this.connection.ma(a,b)};
m.Eb=function(a,b,c){if(this.started&&!this.h()){var d=b||{};switch(a){case "addEventListener":"string"===typeof d.event&&this.addListener(d.event);break;case "removeEventListener":"string"===typeof d.event&&this.removeListener(d.event);break;default:this.api.isReady()&&this.api.isExternalMethodAvailable(a,c||null)&&(b=js(a,b||{}),c=this.api.handleExternalCall(a,b,c||null),(c=ks(a,c))&&this.ma(a,c))}}};
m.addListener=function(a){if(!(a in this.i)){var b=this.lc.bind(this,a);this.i[a]=b;this.addEventListener(a,b)}};
m.lc=function(a,b){this.started&&!this.h()&&this.connection.ma(a,this.Xa(a,b))};
m.Xa=function(a,b){if(null!=b)return{value:b}};
m.removeListener=function(a){a in this.i&&(this.removeEventListener(a,this.i[a]),delete this.i[a])};
m.I=function(){var a=this.connection;a.h()||ag(a.i,"command",this.Eb,this);this.connection=null;for(var b in this.i)this.i.hasOwnProperty(b)&&this.removeListener(b);I.prototype.I.call(this)};function ls(a,b){is.call(this,b);this.api=a;this.start()}
r(ls,is);ls.prototype.addEventListener=function(a,b){this.api.addEventListener(a,b)};
ls.prototype.removeEventListener=function(a,b){this.api.removeEventListener(a,b)};
function js(a,b){switch(a){case "loadVideoById":return a=fs(b),[a];case "cueVideoById":return a=fs(b),[a];case "loadVideoByPlayerVars":return[b];case "cueVideoByPlayerVars":return[b];case "loadPlaylist":return a=gs(b),[a];case "cuePlaylist":return a=gs(b),[a];case "seekTo":return[b.seconds,b.allowSeekAhead];case "playVideoAt":return[b.index];case "setVolume":return[b.volume];case "setPlaybackQuality":return[b.suggestedQuality];case "setPlaybackRate":return[b.suggestedRate];case "setLoop":return[b.loopPlaylists];
case "setShuffle":return[b.shufflePlaylist];case "getOptions":return[b.module];case "getOption":return[b.module,b.option];case "setOption":return[b.module,b.option,b.value];case "handleGlobalKeyDown":return[b.keyCode,b.shiftKey,b.ctrlKey,b.altKey,b.metaKey,b.key,b.code]}return[]}
function ks(a,b){switch(a){case "isMuted":return{muted:b};case "getVolume":return{volume:b};case "getPlaybackRate":return{playbackRate:b};case "getAvailablePlaybackRates":return{availablePlaybackRates:b};case "getVideoLoadedFraction":return{videoLoadedFraction:b};case "getPlayerState":return{playerState:b};case "getCurrentTime":return{currentTime:b};case "getPlaybackQuality":return{playbackQuality:b};case "getAvailableQualityLevels":return{availableQualityLevels:b};case "getDuration":return{duration:b};
case "getVideoUrl":return{videoUrl:b};case "getVideoEmbedCode":return{videoEmbedCode:b};case "getPlaylist":return{playlist:b};case "getPlaylistIndex":return{playlistIndex:b};case "getOptions":return{options:b};case "getOption":return{option:b}}}
ls.prototype.Xa=function(a,b){switch(a){case "onReady":return;case "onStateChange":return{playerState:b};case "onPlaybackQualityChange":return{playbackQuality:b};case "onPlaybackRateChange":return{playbackRate:b};case "onError":return{errorCode:b}}return is.prototype.Xa.call(this,a,b)};
ls.prototype.I=function(){is.prototype.I.call(this);delete this.api};function ms(a){a=void 0===a?!1:a;I.call(this);this.i=new J(a);de(this,Wa(be,this.i))}
Ya(ms,I);ms.prototype.subscribe=function(a,b,c){return this.h()?0:this.i.subscribe(a,b,c)};
ms.prototype.l=function(a,b){this.h()||this.i.ka.apply(this.i,arguments)};function ns(a,b,c){ms.call(this);this.j=a;this.destination=b;this.id=c}
r(ns,ms);ns.prototype.ma=function(a,b){this.h()||this.j.ma(this.destination,this.id,a,b)};
ns.prototype.I=function(){this.destination=this.j=null;ms.prototype.I.call(this)};function os(a,b,c){I.call(this);this.destination=a;this.origin=c;this.i=Oh(window,"message",this.j.bind(this));this.connection=new ns(this,a,b);de(this,Wa(be,this.connection))}
r(os,I);os.prototype.ma=function(a,b,c,d){this.h()||a!==this.destination||(a={id:b,command:c},d&&(a.data=d),this.destination.postMessage(Cf(a),this.origin))};
os.prototype.j=function(a){var b;if(b=!this.h())if(b=a.origin===this.origin)a:{b=this.destination;do{b:{var c=a.source;do{if(c===b){c=!0;break b}if(c===c.parent)break;c=c.parent}while(null!=c);c=!1}if(c){b=!0;break a}b=b.opener}while(null!=b);b=!1}if(b&&(b=a.data,"string"===typeof b)){try{b=JSON.parse(b)}catch(d){return}b.command&&(c=this.connection,c.h()||c.l("command",b.command,b.data,a.origin))}};
os.prototype.I=function(){Ph(this.i);this.destination=null;I.prototype.I.call(this)};function ps(a){a=a||{};var b={},c={};this.url=a.url||"";this.args=a.args||ub(b);this.assets=a.assets||{};this.attrs=a.attrs||ub(c);this.fallback=a.fallback||null;this.fallbackMessage=a.fallbackMessage||null;this.html5=!!a.html5;this.disable=a.disable||{};this.loaded=!!a.loaded;this.messages=a.messages||{}}
ps.prototype.clone=function(){var a=new ps,b;for(b in this)if(this.hasOwnProperty(b)){var c=this[b];"object"==Na(c)?a[b]=ub(c):a[b]=c}return a};var qs=/cssbin\/(?:debug-)?([a-zA-Z0-9_-]+?)(?:-2x|-web|-rtl|-vfl|.css)/;function rs(a){a=a||"";if(window.spf){var b=a.match(qs);spf.style.load(a,b?b[1]:"",void 0)}else ss(a)}
function ss(a){var b=ts(a),c=document.getElementById(b),d=c&&nr(c,"loaded");d||c&&!d||(c=us(a,b,function(){nr(c,"loaded")||(lr(c),Si(b),Rh(Wa(Ti,b),0))}))}
function us(a,b,c){var d=document.createElement("link");d.id=b;d.onload=function(){c&&setTimeout(c,0)};
a=Af(a);Tb(d,a);(document.getElementsByTagName("head")[0]||document.body).appendChild(d);return d}
function ts(a){var b=Ad("A");Cb("This URL is never added to the DOM");Sb(b,new Kb(a,Lb));a=b.href.replace(/^[a-zA-Z]+:\/\//,"//");return"css-"+Xb(a)}
;function vs(){I.call(this);this.i=[]}
r(vs,I);vs.prototype.I=function(){for(;this.i.length;){var a=this.i.pop();a.target.removeEventListener(a.name,a.la,void 0)}I.prototype.I.call(this)};function ws(){vs.apply(this,arguments)}
r(ws,vs);function xs(a,b,c,d){I.call(this);var e=this;this.M=b;this.webPlayerContextConfig=d;this.Ua=!1;this.api={};this.Ea=this.u=null;this.S=new J;this.i={};this.ga=this.Fa=this.elementId=this.Va=this.config=null;this.Z=!1;this.l=this.A=null;this.Ga={};this.Lb=["onReady"];this.lastError=null;this.mb=NaN;this.L={};this.Mb=new ws(this);this.oa=0;this.j=this.m=a;de(this,Wa(be,this.S));ys(this);zs(this);de(this,Wa(be,this.Mb));c?this.oa=Rh(function(){e.loadNewVideoConfig(c)},0):d&&(As(this),Bs(this))}
r(xs,I);m=xs.prototype;m.getId=function(){return this.M};
m.loadNewVideoConfig=function(a){if(!this.h()){this.oa&&(Sh(this.oa),this.oa=0);var b=a||{};b instanceof ps||(b=new ps(b));this.config=b;this.setConfig(a);Bs(this);this.isReady()&&Cs(this)}};
function As(a){var b,c;a.webPlayerContextConfig?c=a.webPlayerContextConfig.rootElementId:c=a.config.attrs.id;a.elementId=c||a.elementId;"video-player"===a.elementId&&(a.elementId=a.M,a.webPlayerContextConfig?a.webPlayerContextConfig.rootElementId=a.M:a.config.attrs.id=a.M);(null===(b=a.j)||void 0===b?void 0:b.id)===a.elementId&&(a.elementId+="-player",a.webPlayerContextConfig?a.webPlayerContextConfig.rootElementId=a.elementId:a.config.attrs.id=a.elementId)}
m.setConfig=function(a){var b;this.Va=a;this.config=Ds(a);As(this);this.Fa||(this.Fa=Es(this,(null===(b=this.config.args)||void 0===b?void 0:b.jsapicallback)||"onYouTubePlayerReady"));this.config.args?this.config.args.jsapicallback=null:this.config.args={jsapicallback:null};var c;if(null===(c=this.config)||void 0===c?0:c.attrs)a=this.config.attrs,(c=a.width)&&this.j&&(this.j.style.width=Md(Number(c)||c)),(a=a.height)&&this.j&&(this.j.style.height=Md(Number(a)||a))};
function Cs(a){var b;a.config&&!0!==a.config.loaded&&(a.config.loaded=!0,!a.config.args||"0"!==a.config.args.autoplay&&0!==a.config.args.autoplay&&!1!==a.config.args.autoplay?a.api.loadVideoByPlayerVars(null!==(b=a.config.args)&&void 0!==b?b:null):a.api.cueVideoByPlayerVars(a.config.args))}
function Fs(a){var b=!0,c=Gs(a);c&&a.config&&(a=Hs(a),b=nr(c,"version")===a);return b&&!!A("yt.player.Application.create")}
function Bs(a){if(!a.h()&&!a.Z){var b=Fs(a);if(b&&"html5"===(Gs(a)?"html5":null))a.ga="html5",a.isReady()||Is(a);else if(Js(a),a.ga="html5",b&&a.l&&a.m)a.m.appendChild(a.l),Is(a);else{a.config&&(a.config.loaded=!0);var c=!1;a.A=function(){c=!0;var d=Ks(a,"player_bootstrap_method")?A("yt.player.Application.createAlternate")||A("yt.player.Application.create"):A("yt.player.Application.create");var e=a.config?Ds(a.config):void 0;d&&d(a.m,e,a.webPlayerContextConfig);Is(a)};
a.Z=!0;b?a.A():(rr(Hs(a),a.A),(b=Ls(a))&&rs(b),Ms(a)&&!c&&z("yt.player.Application.create",null,void 0))}}}
function Gs(a){var b=zd(a.elementId);!b&&a.j&&a.j.querySelector&&(b=a.j.querySelector("#"+a.elementId));return b}
function Is(a){var b;if(!a.h()){var c=Gs(a),d=!1;c&&c.getApiInterface&&c.getApiInterface()&&(d=!0);d?(a.Z=!1,!Ks(a,"html5_remove_not_servable_check_killswitch")&&(null===c||void 0===c?0:c.isNotServable)&&a.config&&(null===c||void 0===c?0:c.isNotServable(null===(b=a.config.args)||void 0===b?void 0:b.video_id))||Ns(a)):a.mb=Rh(function(){Is(a)},50)}}
function Ns(a){ys(a);a.Ua=!0;var b=Gs(a);if(b){a.u=Os(a,b,"addEventListener");a.Ea=Os(a,b,"removeEventListener");var c=b.getApiInterface();c=c.concat(b.getInternalApiInterface());for(var d=a.api,e=0;e<c.length;e++){var f=c[e];d[f]||(d[f]=Os(a,b,f))}}for(var g in a.i)a.i.hasOwnProperty(g)&&a.u&&a.u(g,a.i[g]);Cs(a);a.Fa&&a.Fa(a.api);a.S.ka("onReady",a.api)}
function Os(a,b,c){var d=b[c];return function(){var e=Fa.apply(0,arguments);try{return a.lastError=null,d.apply(b,e)}catch(f){"sendAbandonmentPing"!==c&&(f.params=c,a.lastError=f,Tn(f))}}}
function ys(a){a.Ua=!1;if(a.Ea)for(var b in a.i)a.i.hasOwnProperty(b)&&a.Ea(b,a.i[b]);for(var c in a.L)a.L.hasOwnProperty(c)&&Sh(Number(c));a.L={};a.u=null;a.Ea=null;b=a.api;for(var d in b)b.hasOwnProperty(d)&&(b[d]=null);b.addEventListener=function(e,f){a.addEventListener(e,f)};
b.removeEventListener=function(e,f){a.removeEventListener(e,f)};
b.destroy=function(){a.dispose()};
b.getLastError=function(){return a.getLastError()};
b.getPlayerType=function(){return a.getPlayerType()};
b.getCurrentVideoConfig=function(){return a.Va};
b.loadNewVideoConfig=function(e){a.loadNewVideoConfig(e)};
b.isReady=function(){return a.isReady()}}
m.isReady=function(){return this.Ua};
function zs(a){a.addEventListener("WATCH_LATER_VIDEO_ADDED",function(b){Si("WATCH_LATER_VIDEO_ADDED",b)});
a.addEventListener("WATCH_LATER_VIDEO_REMOVED",function(b){Si("WATCH_LATER_VIDEO_REMOVED",b)});
a.addEventListener("onAdAnnounce",function(b){Si("a11y-announce",b)})}
m.addEventListener=function(a,b){var c=this,d=Es(this,b);d&&(0<=eb(this.Lb,a)||this.i[a]||(b=Ps(this,a),this.u&&this.u(a,b)),this.S.subscribe(a,d),"onReady"===a&&this.isReady()&&Rh(function(){d(c.api)},0))};
m.removeEventListener=function(a,b){this.h()||(b=Es(this,b))&&ag(this.S,a,b)};
function Es(a,b){var c=b;if("string"===typeof b){if(a.Ga[b])return a.Ga[b];c=function(){var d=Fa.apply(0,arguments),e=A(b);if(e)try{e.apply(x,d)}catch(f){Sn(f)}};
a.Ga[b]=c}return c?c:null}
function Ps(a,b){var c="ytPlayer"+b+a.M;a.i[b]=c;x[c]=function(d){var e=Rh(function(){if(!a.h()){a.S.ka(b,null!==d&&void 0!==d?d:void 0);var f=a.L,g=String(e);g in f&&delete f[g]}},0);
rb(a.L,String(e))};
return c}
m.getPlayerType=function(){return this.ga||(Gs(this)?"html5":null)};
m.getLastError=function(){return this.lastError};
function Js(a){a.cancel();ys(a);a.ga=null;a.config&&(a.config.loaded=!1);var b=Gs(a);b&&(Fs(a)||!Ms(a)?a.l=b:(b&&b.destroy&&b.destroy(),a.l=null));if(a.m)for(a=a.m;b=a.firstChild;)a.removeChild(b)}
m.cancel=function(){this.A&&xr(Hs(this),this.A);Sh(this.mb);this.Z=!1};
m.I=function(){Js(this);if(this.l&&this.config&&this.l.destroy)try{this.l.destroy()}catch(b){Sn(b)}this.Ga=null;for(var a in this.i)this.i.hasOwnProperty(a)&&(x[this.i[a]]=null);this.Va=this.config=this.api=null;delete this.m;delete this.j;I.prototype.I.call(this)};
function Ms(a){var b,c;a=null===(c=null===(b=a.config)||void 0===b?void 0:b.args)||void 0===c?void 0:c.fflags;return!!a&&-1!==a.indexOf("player_destroy_old_version=true")}
function Hs(a){return a.webPlayerContextConfig?a.webPlayerContextConfig.jsUrl:(a=a.config.assets)?a.js:""}
function Ls(a){return a.webPlayerContextConfig?a.webPlayerContextConfig.cssUrl:(a=a.config.assets)?a.css:""}
function Ks(a,b){var c;if(a.webPlayerContextConfig)var d=a.webPlayerContextConfig.serializedExperimentFlags;else if(null===(c=a.config)||void 0===c?0:c.args)d=a.config.args.fflags;return"true"===li(d||"","&")[b]}
function Ds(a){for(var b={},c=q(Object.keys(a)),d=c.next();!d.done;d=c.next()){d=d.value;var e=a[d];b[d]="object"===typeof e?ub(e):e}return b}
;var Qs={},Rs="player_uid_"+(1E9*Math.random()>>>0);function Ss(a,b,c){var d="player";c=void 0===c?!0:c;d="string"===typeof d?zd(d):d;var e=Rs+"_"+Qa(d),f=Qs[e];if(f&&c)return Ts(a,b)?f.api.loadVideoByPlayerVars(a.args||null):f.loadNewVideoConfig(a),f.api;f=new xs(d,e,a,b);Qs[e]=f;Si("player-added",f.api);de(f,function(){delete Qs[f.getId()]});
return f.api}
function Ts(a,b){return b&&b.serializedExperimentFlags?b.serializedExperimentFlags.includes("web_player_remove_playerproxy=true"):a&&a.args&&a.args.fflags?a.args.fflags.includes("web_player_remove_playerproxy=true"):!1}
;var Us=null,Vs=null,Ws=null;function Xs(){var a=Us.getVideoData(1);a=a.title?a.title+" - YouTube":"YouTube";document.title!==a&&(document.title=a)}
;function Ys(a,b,c){a="ST-"+Xb(a).toString(36);b=b?cc(b):"";c=c||5;yo()&&Ij(a,b,c)}
;function Zs(a,b,c){b=void 0===b?{}:b;c=void 0===c?!1:c;var d=B("EVENT_ID");d&&(b.ei||(b.ei=d));if(b){d=a;var e=void 0===e?!0:e;var f=B("VALID_SESSION_TEMPDATA_DOMAINS",[]),g=$b(window.location.href);g&&f.push(g);g=$b(d);if(0<=eb(f,g)||!g&&0==d.lastIndexOf("/",0))if(K("autoescape_tempdata_url")&&(f=document.createElement("a"),Sb(f,d),d=f.href),d&&(d=ac(d),f=d.indexOf("#"),d=0>f?d:d.substr(0,f)))if(e&&!b.csn&&(b.itct||b.ved)&&(b=Object.assign({csn:ko()},b)),h){var h=parseInt(h,10);isFinite(h)&&0<h&&
Ys(d,b,h)}else Ys(d,b)}if(c)return!1;if((window.ytspf||{}).enabled)spf.navigate(a);else{var k=void 0===k?{}:k;var l=void 0===l?"":l;var n=void 0===n?window:n;c=n.location;a=dc(a,k)+l;var t=void 0===t?Ld:t;a:{t=void 0===t?Ld:t;for(k=0;k<t.length;++k)if(l=t[k],l instanceof Jd&&l.isValid(a)){t=new td(a,rd);break a}t=void 0}c.href=vd(t||ud)}return!0}
;z("yt.setConfig",uh,void 0);z("yt.config.set",uh,void 0);z("yt.setMsg",po,void 0);z("yt.msgs.set",po,void 0);z("yt.logging.errors.log",Sn,void 0);
z("writeEmbed",function(){var a=B("PLAYER_CONFIG",void 0);if(!a){var b=B("PLAYER_VARS",void 0);b&&(a={args:b})}Go(!0);"gvn"===a.args.ps&&(document.body.style.backgroundColor="transparent");a.attrs||(a.attrs={width:"100%",height:"100%",id:"video-player"});var c=document.referrer;b=B("POST_MESSAGE_ORIGIN");window!==window.top&&c&&c!==document.URL&&(a.args.loaderUrl=c);K("embeds_js_api_set_1p_cookie")&&(c=qi(window.location.href),c.embedsTokenValue&&(a.args.embedsTokenValue=c.embedsTokenValue));gq();
if((c=B("WEB_PLAYER_CONTEXT_CONFIGS",void 0))&&"WEB_PLAYER_CONTEXT_CONFIG_ID_EMBEDDED_PLAYER"in c){c=c.WEB_PLAYER_CONTEXT_CONFIG_ID_EMBEDDED_PLAYER;if(!c.serializedForcedExperimentIds){var d=qi(window.location.href);d.forced_experiments&&(c.serializedForcedExperimentIds=d.forced_experiments)}Us=Ss(a,c,!1)}else Us=Ss(a);Us.addEventListener("onVideoDataChange",Xs);a=B("POST_MESSAGE_ID","player");B("ENABLE_JS_API")?Ws=new hs(Us):B("ENABLE_POST_API")&&"string"===typeof a&&"string"===typeof b&&(Vs=new os(window.parent,
a,b),Ws=new ls(Us,Vs.connection));Ar();K("ytidb_create_logger_embed_killswitch")||tk();K("flush_gel_on_teardown")&&(a={},Sr||(Sr=new Rr),Sr.install((a.flush_logs={la:function(){kj()}},a)));
K("networkless_logging_web_embedded")&&(K("embeds_web_enable_new_nwl")?cn():ln());K("embeds_enable_ua_ch_polyfill")&&$r();K("ytidb_clear_embedded_player")&&ai.N(function(){if(!Kq){var e={pb:{feedbackEndpoint:Xo(Eq),modifyChannelNotificationPreferenceEndpoint:Xo(Fq),playlistEditEndpoint:Xo(Gq),subscribeEndpoint:Xo(Cq),unsubscribeEndpoint:Xo(Dq),webPlayerShareEntityServiceEndpoint:Xo(Hq)}},f=K("web_enable_client_location_service")?Ro.getInstance():void 0,g={};f&&(g.client_location=f);if(void 0===h){Ho.h||
(Ho.h=new Ho);var h=Ho.h}if(void 0===k){Jq.h||(Jq.h=new Jq);var k=Jq.h}rq(e,k,h,g);Kq=qq.h}kr()})},void 0);
var $s=Gh(function(){nq();var a=Mj.getInstance(),b=Pj(119),c=1<window.devicePixelRatio;if(document.body&&kf(document.body,"exp-invert-logo"))if(c&&!kf(document.body,"inverted-hdpi")){var d=document.body;if(d.classList)d.classList.add("inverted-hdpi");else if(!kf(d,"inverted-hdpi")){var e=hf(d);jf(d,e+(0<e.length?" inverted-hdpi":"inverted-hdpi"))}}else!c&&kf(document.body,"inverted-hdpi")&&lf();if(b!=c){b="f"+(Math.floor(119/31)+1);d=Qj(b)||0;d=c?d|67108864:d&-67108865;0==d?delete Lj[b]:(c=d.toString(16),
Lj[b]=c.toString());c=!0;K("web_secure_pref_cookie_killswitch")&&(c=!1);b=a.h;d=[];for(var f in Lj)d.push(f+"="+encodeURIComponent(String(Lj[f])));Ij(b,d.join("&"),63072E3,a.i,c)}Uq.h||(Uq.h=new Uq);a=Uq.h;f=16623;var g=void 0===g?{}:g;Object.values(qo).includes(f)||(Tn(new N("createClientScreen() called with a non-page VE",f)),f=83769);g.isHistoryNavigation||a.h.push({rootVe:f,key:g.key||""});a.o=[];a.D=[];g.sb?Xq(a,f,g):Yq(a,f,g)}),at=Gh(function(){Us&&Us.sendAbandonmentPing&&Us.sendAbandonmentPing();
B("PL_ATT")&&Gr.dispose();for(var a=0,b=yr.length;a<b;a++)ai.U(yr[a]);yr.length=0;wr("//static.doubleclick.net/instream/ad_status.js");zr=!1;uh("DCLKSTAT",0);ce(Ws,Vs);Us&&(Us.removeEventListener("onVideoDataChange",Xs),Us.destroy())});
window.addEventListener?(window.addEventListener("load",$s),window.addEventListener("unload",at)):window.attachEvent&&(window.attachEvent("onload",$s),window.attachEvent("onunload",at));Xa("yt.abuse.player.botguardInitialized",A("yt.abuse.player.botguardInitialized")||Hr);Xa("yt.abuse.player.invokeBotguard",A("yt.abuse.player.invokeBotguard")||Ir);Xa("yt.abuse.dclkstatus.checkDclkStatus",A("yt.abuse.dclkstatus.checkDclkStatus")||Br);
Xa("yt.player.exports.navigate",A("yt.player.exports.navigate")||Zs);Xa("yt.util.activity.init",A("yt.util.activity.init")||ci);Xa("yt.util.activity.getTimeSinceActive",A("yt.util.activity.getTimeSinceActive")||fi);Xa("yt.util.activity.setTimestamp",A("yt.util.activity.setTimestamp")||di);}).call(this);
