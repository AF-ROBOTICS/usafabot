(function(){/*

 Copyright The Closure Library Authors.
 SPDX-License-Identifier: Apache-2.0
*/
'use strict';var m;function aa(a){var b=0;return function(){return b<a.length?{done:!1,value:a[b++]}:{done:!0}}}
var ba="function"==typeof Object.defineProperties?Object.defineProperty:function(a,b,c){if(a==Array.prototype||a==Object.prototype)return a;a[b]=c.value;return a};
function ca(a){a=["object"==typeof globalThis&&globalThis,a,"object"==typeof window&&window,"object"==typeof self&&self,"object"==typeof global&&global];for(var b=0;b<a.length;++b){var c=a[b];if(c&&c.Math==Math)return c}throw Error("Cannot find global object");}
var da=ca(this);function p(a,b){if(b)a:{var c=da;a=a.split(".");for(var d=0;d<a.length-1;d++){var e=a[d];if(!(e in c))break a;c=c[e]}a=a[a.length-1];d=c[a];b=b(d);b!=d&&null!=b&&ba(c,a,{configurable:!0,writable:!0,value:b})}}
p("Symbol",function(a){function b(f){if(this instanceof b)throw new TypeError("Symbol is not a constructor");return new c(d+(f||"")+"_"+e++,f)}
function c(f,g){this.h=f;ba(this,"description",{configurable:!0,writable:!0,value:g})}
if(a)return a;c.prototype.toString=function(){return this.h};
var d="jscomp_symbol_"+(1E9*Math.random()>>>0)+"_",e=0;return b});
p("Symbol.iterator",function(a){if(a)return a;a=Symbol("Symbol.iterator");for(var b="Array Int8Array Uint8Array Uint8ClampedArray Int16Array Uint16Array Int32Array Uint32Array Float32Array Float64Array".split(" "),c=0;c<b.length;c++){var d=da[b[c]];"function"===typeof d&&"function"!=typeof d.prototype[a]&&ba(d.prototype,a,{configurable:!0,writable:!0,value:function(){return fa(aa(this))}})}return a});
function fa(a){a={next:a};a[Symbol.iterator]=function(){return this};
return a}
function q(a){var b="undefined"!=typeof Symbol&&Symbol.iterator&&a[Symbol.iterator];return b?b.call(a):{next:aa(a)}}
function ha(a){if(!(a instanceof Array)){a=q(a);for(var b,c=[];!(b=a.next()).done;)c.push(b.value);a=c}return a}
var ja="function"==typeof Object.create?Object.create:function(a){function b(){}
b.prototype=a;return new b},ka=function(){function a(){function c(){}
new c;Reflect.construct(c,[],function(){});
return new c instanceof c}
if("undefined"!=typeof Reflect&&Reflect.construct){if(a())return Reflect.construct;var b=Reflect.construct;return function(c,d,e){c=b(c,d);e&&Reflect.setPrototypeOf(c,e.prototype);return c}}return function(c,d,e){void 0===e&&(e=c);
e=ja(e.prototype||Object.prototype);return Function.prototype.apply.call(c,e,d)||e}}(),la;
if("function"==typeof Object.setPrototypeOf)la=Object.setPrototypeOf;else{var ma;a:{var na={a:!0},oa={};try{oa.__proto__=na;ma=oa.a;break a}catch(a){}ma=!1}la=ma?function(a,b){a.__proto__=b;if(a.__proto__!==b)throw new TypeError(a+" is not extensible");return a}:null}var pa=la;
function r(a,b){a.prototype=ja(b.prototype);a.prototype.constructor=a;if(pa)pa(a,b);else for(var c in b)if("prototype"!=c)if(Object.defineProperties){var d=Object.getOwnPropertyDescriptor(b,c);d&&Object.defineProperty(a,c,d)}else a[c]=b[c];a.aa=b.prototype}
function qa(){this.D=!1;this.l=null;this.i=void 0;this.h=1;this.o=this.m=0;this.A=this.j=null}
function ra(a){if(a.D)throw new TypeError("Generator is already running");a.D=!0}
qa.prototype.u=function(a){this.i=a};
function sa(a,b){a.j={vb:b,zb:!0};a.h=a.m||a.o}
qa.prototype.return=function(a){this.j={return:a};this.h=this.o};
function v(a,b,c){a.h=c;return{value:b}}
qa.prototype.s=function(a){this.h=a};
function ta(a,b,c){a.m=b;void 0!=c&&(a.o=c)}
function va(a,b){a.h=b;a.m=0}
function wa(a){a.m=0;var b=a.j.vb;a.j=null;return b}
function xa(a){a.A=[a.j];a.m=0;a.o=0}
function ya(a){var b=a.A.splice(0)[0];(b=a.j=a.j||b)?b.zb?a.h=a.m||a.o:void 0!=b.s&&a.o<b.s?(a.h=b.s,a.j=null):a.h=a.o:a.h=0}
function za(a){this.h=new qa;this.i=a}
function Aa(a,b){ra(a.h);var c=a.h.l;if(c)return Ba(a,"return"in c?c["return"]:function(d){return{value:d,done:!0}},b,a.h.return);
a.h.return(b);return Ca(a)}
function Ba(a,b,c,d){try{var e=b.call(a.h.l,c);if(!(e instanceof Object))throw new TypeError("Iterator result "+e+" is not an object");if(!e.done)return a.h.D=!1,e;var f=e.value}catch(g){return a.h.l=null,sa(a.h,g),Ca(a)}a.h.l=null;d.call(a.h,f);return Ca(a)}
function Ca(a){for(;a.h.h;)try{var b=a.i(a.h);if(b)return a.h.D=!1,{value:b.value,done:!1}}catch(c){a.h.i=void 0,sa(a.h,c)}a.h.D=!1;if(a.h.j){b=a.h.j;a.h.j=null;if(b.zb)throw b.vb;return{value:b.return,done:!0}}return{value:void 0,done:!0}}
function Da(a){this.next=function(b){ra(a.h);a.h.l?b=Ba(a,a.h.l.next,b,a.h.u):(a.h.u(b),b=Ca(a));return b};
this.throw=function(b){ra(a.h);a.h.l?b=Ba(a,a.h.l["throw"],b,a.h.u):(sa(a.h,b),b=Ca(a));return b};
this.return=function(b){return Aa(a,b)};
this[Symbol.iterator]=function(){return this}}
function Ea(a){function b(d){return a.next(d)}
function c(d){return a.throw(d)}
return new Promise(function(d,e){function f(g){g.done?d(g.value):Promise.resolve(g.value).then(b,c).then(f,e)}
f(a.next())})}
function w(a){return Ea(new Da(new za(a)))}
function Fa(){for(var a=Number(this),b=[],c=a;c<arguments.length;c++)b[c-a]=arguments[c];return b}
p("Reflect",function(a){return a?a:{}});
p("Reflect.construct",function(){return ka});
p("Reflect.setPrototypeOf",function(a){return a?a:pa?function(b,c){try{return pa(b,c),!0}catch(d){return!1}}:null});
p("Promise",function(a){function b(g){this.h=0;this.j=void 0;this.i=[];this.D=!1;var h=this.l();try{g(h.resolve,h.reject)}catch(k){h.reject(k)}}
function c(){this.h=null}
function d(g){return g instanceof b?g:new b(function(h){h(g)})}
if(a)return a;c.prototype.i=function(g){if(null==this.h){this.h=[];var h=this;this.j(function(){h.o()})}this.h.push(g)};
var e=da.setTimeout;c.prototype.j=function(g){e(g,0)};
c.prototype.o=function(){for(;this.h&&this.h.length;){var g=this.h;this.h=[];for(var h=0;h<g.length;++h){var k=g[h];g[h]=null;try{k()}catch(l){this.l(l)}}}this.h=null};
c.prototype.l=function(g){this.j(function(){throw g;})};
b.prototype.l=function(){function g(l){return function(n){k||(k=!0,l.call(h,n))}}
var h=this,k=!1;return{resolve:g(this.S),reject:g(this.o)}};
b.prototype.S=function(g){if(g===this)this.o(new TypeError("A Promise cannot resolve to itself"));else if(g instanceof b)this.ga(g);else{a:switch(typeof g){case "object":var h=null!=g;break a;case "function":h=!0;break a;default:h=!1}h?this.M(g):this.m(g)}};
b.prototype.M=function(g){var h=void 0;try{h=g.then}catch(k){this.o(k);return}"function"==typeof h?this.oa(h,g):this.m(g)};
b.prototype.o=function(g){this.u(2,g)};
b.prototype.m=function(g){this.u(1,g)};
b.prototype.u=function(g,h){if(0!=this.h)throw Error("Cannot settle("+g+", "+h+"): Promise already settled in state"+this.h);this.h=g;this.j=h;2===this.h&&this.Z();this.A()};
b.prototype.Z=function(){var g=this;e(function(){if(g.L()){var h=da.console;"undefined"!==typeof h&&h.error(g.j)}},1)};
b.prototype.L=function(){if(this.D)return!1;var g=da.CustomEvent,h=da.Event,k=da.dispatchEvent;if("undefined"===typeof k)return!0;"function"===typeof g?g=new g("unhandledrejection",{cancelable:!0}):"function"===typeof h?g=new h("unhandledrejection",{cancelable:!0}):(g=da.document.createEvent("CustomEvent"),g.initCustomEvent("unhandledrejection",!1,!0,g));g.promise=this;g.reason=this.j;return k(g)};
b.prototype.A=function(){if(null!=this.i){for(var g=0;g<this.i.length;++g)f.i(this.i[g]);this.i=null}};
var f=new c;b.prototype.ga=function(g){var h=this.l();g.Ja(h.resolve,h.reject)};
b.prototype.oa=function(g,h){var k=this.l();try{g.call(h,k.resolve,k.reject)}catch(l){k.reject(l)}};
b.prototype.then=function(g,h){function k(z,u){return"function"==typeof z?function(D){try{l(z(D))}catch(E){n(E)}}:u}
var l,n,t=new b(function(z,u){l=z;n=u});
this.Ja(k(g,l),k(h,n));return t};
b.prototype.catch=function(g){return this.then(void 0,g)};
b.prototype.Ja=function(g,h){function k(){switch(l.h){case 1:g(l.j);break;case 2:h(l.j);break;default:throw Error("Unexpected state: "+l.h);}}
var l=this;null==this.i?f.i(k):this.i.push(k);this.D=!0};
b.resolve=d;b.reject=function(g){return new b(function(h,k){k(g)})};
b.race=function(g){return new b(function(h,k){for(var l=q(g),n=l.next();!n.done;n=l.next())d(n.value).Ja(h,k)})};
b.all=function(g){var h=q(g),k=h.next();return k.done?d([]):new b(function(l,n){function t(D){return function(E){z[D]=E;u--;0==u&&l(z)}}
var z=[],u=0;do z.push(void 0),u++,d(k.value).Ja(t(z.length-1),n),k=h.next();while(!k.done)})};
return b});
function Ga(a,b){return Object.prototype.hasOwnProperty.call(a,b)}
p("WeakMap",function(a){function b(k){this.h=(h+=Math.random()+1).toString();if(k){k=q(k);for(var l;!(l=k.next()).done;)l=l.value,this.set(l[0],l[1])}}
function c(){}
function d(k){var l=typeof k;return"object"===l&&null!==k||"function"===l}
function e(k){if(!Ga(k,g)){var l=new c;ba(k,g,{value:l})}}
function f(k){var l=Object[k];l&&(Object[k]=function(n){if(n instanceof c)return n;Object.isExtensible(n)&&e(n);return l(n)})}
if(function(){if(!a||!Object.seal)return!1;try{var k=Object.seal({}),l=Object.seal({}),n=new a([[k,2],[l,3]]);if(2!=n.get(k)||3!=n.get(l))return!1;n.delete(k);n.set(l,4);return!n.has(k)&&4==n.get(l)}catch(t){return!1}}())return a;
var g="$jscomp_hidden_"+Math.random();f("freeze");f("preventExtensions");f("seal");var h=0;b.prototype.set=function(k,l){if(!d(k))throw Error("Invalid WeakMap key");e(k);if(!Ga(k,g))throw Error("WeakMap key fail: "+k);k[g][this.h]=l;return this};
b.prototype.get=function(k){return d(k)&&Ga(k,g)?k[g][this.h]:void 0};
b.prototype.has=function(k){return d(k)&&Ga(k,g)&&Ga(k[g],this.h)};
b.prototype.delete=function(k){return d(k)&&Ga(k,g)&&Ga(k[g],this.h)?delete k[g][this.h]:!1};
return b});
p("Map",function(a){function b(){var h={};return h.previous=h.next=h.head=h}
function c(h,k){var l=h.h;return fa(function(){if(l){for(;l.head!=h.h;)l=l.previous;for(;l.next!=l.head;)return l=l.next,{done:!1,value:k(l)};l=null}return{done:!0,value:void 0}})}
function d(h,k){var l=k&&typeof k;"object"==l||"function"==l?f.has(k)?l=f.get(k):(l=""+ ++g,f.set(k,l)):l="p_"+k;var n=h.data_[l];if(n&&Ga(h.data_,l))for(h=0;h<n.length;h++){var t=n[h];if(k!==k&&t.key!==t.key||k===t.key)return{id:l,list:n,index:h,entry:t}}return{id:l,list:n,index:-1,entry:void 0}}
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
function Ha(a,b,c){if(null==a)throw new TypeError("The 'this' value for String.prototype."+c+" must not be null or undefined");if(b instanceof RegExp)throw new TypeError("First argument to String.prototype."+c+" must not be a regular expression");return a+""}
p("String.prototype.endsWith",function(a){return a?a:function(b,c){var d=Ha(this,b,"endsWith");b+="";void 0===c&&(c=d.length);c=Math.max(0,Math.min(c|0,d.length));for(var e=b.length;0<e&&0<c;)if(d[--c]!=b[--e])return!1;return 0>=e}});
p("Array.prototype.find",function(a){return a?a:function(b,c){a:{var d=this;d instanceof String&&(d=String(d));for(var e=d.length,f=0;f<e;f++){var g=d[f];if(b.call(c,g,f,d)){b=g;break a}}b=void 0}return b}});
p("String.prototype.startsWith",function(a){return a?a:function(b,c){var d=Ha(this,b,"startsWith");b+="";var e=d.length,f=b.length;c=Math.max(0,Math.min(c|0,d.length));for(var g=0;g<f&&c<e;)if(d[c++]!=b[g++])return!1;return g>=f}});
function Ia(a,b){a instanceof String&&(a+="");var c=0,d=!1,e={next:function(){if(!d&&c<a.length){var f=c++;return{value:b(f,a[f]),done:!1}}d=!0;return{done:!0,value:void 0}}};
e[Symbol.iterator]=function(){return e};
return e}
p("Array.prototype.entries",function(a){return a?a:function(){return Ia(this,function(b,c){return[b,c]})}});
p("Object.setPrototypeOf",function(a){return a||pa});
var Ja="function"==typeof Object.assign?Object.assign:function(a,b){for(var c=1;c<arguments.length;c++){var d=arguments[c];if(d)for(var e in d)Ga(d,e)&&(a[e]=d[e])}return a};
p("Object.assign",function(a){return a||Ja});
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
p("Object.entries",function(a){return a?a:function(b){var c=[],d;for(d in b)Ga(b,d)&&c.push([d,b[d]]);return c}});
p("Array.prototype.keys",function(a){return a?a:function(){return Ia(this,function(b){return b})}});
p("Array.prototype.values",function(a){return a?a:function(){return Ia(this,function(b,c){return c})}});
p("Object.is",function(a){return a?a:function(b,c){return b===c?0!==b||1/b===1/c:b!==b&&c!==c}});
p("Array.prototype.includes",function(a){return a?a:function(b,c){var d=this;d instanceof String&&(d=String(d));var e=d.length;c=c||0;for(0>c&&(c=Math.max(c+e,0));c<e;c++){var f=d[c];if(f===b||Object.is(f,b))return!0}return!1}});
p("String.prototype.includes",function(a){return a?a:function(b,c){return-1!==Ha(this,b,"includes").indexOf(b,c||0)}});
p("Array.from",function(a){return a?a:function(b,c,d){c=null!=c?c:function(h){return h};
var e=[],f="undefined"!=typeof Symbol&&Symbol.iterator&&b[Symbol.iterator];if("function"==typeof f){b=f.call(b);for(var g=0;!(f=b.next()).done;)e.push(c.call(d,f.value,g++))}else for(f=b.length,g=0;g<f;g++)e.push(c.call(d,b[g],g));return e}});
p("Number.isNaN",function(a){return a?a:function(b){return"number"===typeof b&&isNaN(b)}});
p("Number.MAX_SAFE_INTEGER",function(){return 9007199254740991});
p("Object.values",function(a){return a?a:function(b){var c=[],d;for(d in b)Ga(b,d)&&c.push(b[d]);return c}});
var x=this||self;function y(a,b,c){a=a.split(".");c=c||x;a[0]in c||"undefined"==typeof c.execScript||c.execScript("var "+a[0]);for(var d;a.length&&(d=a.shift());)a.length||void 0===b?c[d]&&c[d]!==Object.prototype[d]?c=c[d]:c=c[d]={}:c[d]=b}
function A(a,b){a=a.split(".");b=b||x;for(var c=0;c<a.length;c++)if(b=b[a[c]],null==b)return null;return b}
function Ka(){}
function La(a){a.eb=void 0;a.getInstance=function(){return a.eb?a.eb:a.eb=new a}}
function Ma(a){var b=typeof a;return"object"!=b?b:a?Array.isArray(a)?"array":b:"null"}
function Na(a){var b=Ma(a);return"array"==b||"object"==b&&"number"==typeof a.length}
function Oa(a){var b=typeof a;return"object"==b&&null!=a||"function"==b}
function Pa(a){return Object.prototype.hasOwnProperty.call(a,Qa)&&a[Qa]||(a[Qa]=++Ra)}
var Qa="closure_uid_"+(1E9*Math.random()>>>0),Ra=0;function Sa(a,b,c){return a.call.apply(a.bind,arguments)}
function Ta(a,b,c){if(!a)throw Error();if(2<arguments.length){var d=Array.prototype.slice.call(arguments,2);return function(){var e=Array.prototype.slice.call(arguments);Array.prototype.unshift.apply(e,d);return a.apply(b,e)}}return function(){return a.apply(b,arguments)}}
function Ua(a,b,c){Function.prototype.bind&&-1!=Function.prototype.bind.toString().indexOf("native code")?Ua=Sa:Ua=Ta;return Ua.apply(null,arguments)}
function Va(a,b){var c=Array.prototype.slice.call(arguments,1);return function(){var d=c.slice();d.push.apply(d,arguments);return a.apply(this,d)}}
function Wa(a,b){y(a,b,void 0)}
function Xa(a,b){function c(){}
c.prototype=b.prototype;a.aa=b.prototype;a.prototype=new c;a.prototype.constructor=a;a.uo=function(d,e,f){for(var g=Array(arguments.length-2),h=2;h<arguments.length;h++)g[h-2]=arguments[h];return b.prototype[e].apply(d,g)}}
function Ya(a){return a}
;function $a(a,b){if(Error.captureStackTrace)Error.captureStackTrace(this,$a);else{var c=Error().stack;c&&(this.stack=c)}a&&(this.message=String(a));void 0!==b&&(this.Ob=b)}
Xa($a,Error);$a.prototype.name="CustomError";function ab(a){a=a.url;var b=/[?&]dsh=1(&|$)/.test(a);this.j=!b&&/[?&]ae=1(&|$)/.test(a);this.l=!b&&/[?&]ae=2(&|$)/.test(a);if((this.h=/[?&]adurl=([^&]*)/.exec(a))&&this.h[1]){try{var c=decodeURIComponent(this.h[1])}catch(d){c=null}this.i=c}}
;function bb(a){var b=!1,c;return function(){b||(c=a(),b=!0);return c}}
;var cb=Array.prototype.indexOf?function(a,b){return Array.prototype.indexOf.call(a,b,void 0)}:function(a,b){if("string"===typeof a)return"string"!==typeof b||1!=b.length?-1:a.indexOf(b,0);
for(var c=0;c<a.length;c++)if(c in a&&a[c]===b)return c;return-1},db=Array.prototype.forEach?function(a,b,c){Array.prototype.forEach.call(a,b,c)}:function(a,b,c){for(var d=a.length,e="string"===typeof a?a.split(""):a,f=0;f<d;f++)f in e&&b.call(c,e[f],f,a)},eb=Array.prototype.filter?function(a,b){return Array.prototype.filter.call(a,b,void 0)}:function(a,b){for(var c=a.length,d=[],e=0,f="string"===typeof a?a.split(""):a,g=0;g<c;g++)if(g in f){var h=f[g];
b.call(void 0,h,g,a)&&(d[e++]=h)}return d},fb=Array.prototype.map?function(a,b){return Array.prototype.map.call(a,b,void 0)}:function(a,b){for(var c=a.length,d=Array(c),e="string"===typeof a?a.split(""):a,f=0;f<c;f++)f in e&&(d[f]=b.call(void 0,e[f],f,a));
return d},gb=Array.prototype.reduce?function(a,b,c){return Array.prototype.reduce.call(a,b,c)}:function(a,b,c){var d=c;
db(a,function(e,f){d=b.call(void 0,d,e,f,a)});
return d};
function hb(a,b){a:{for(var c=a.length,d="string"===typeof a?a.split(""):a,e=0;e<c;e++)if(e in d&&b.call(void 0,d[e],e,a)){b=e;break a}b=-1}return 0>b?null:"string"===typeof a?a.charAt(b):a[b]}
function ib(a,b){b=cb(a,b);var c;(c=0<=b)&&Array.prototype.splice.call(a,b,1);return c}
function kb(a,b){for(var c=1;c<arguments.length;c++){var d=arguments[c];if(Na(d)){var e=a.length||0,f=d.length||0;a.length=e+f;for(var g=0;g<f;g++)a[e+g]=d[g]}else a.push(d)}}
;function lb(a,b){for(var c in a)b.call(void 0,a[c],c,a)}
function mb(a){var b=nb,c;for(c in b)if(a.call(void 0,b[c],c,b))return c}
function ob(a){for(var b in a)return!1;return!0}
function pb(a,b){if(null!==a&&b in a)throw Error('The object already contains the key "'+b+'"');a[b]=!0}
function qb(){var a=B("PLAYER_VARS",{});return null!==a&&"privembed"in a?a.privembed:!1}
function rb(a,b){for(var c in a)if(!(c in b)||a[c]!==b[c])return!1;for(var d in b)if(!(d in a))return!1;return!0}
function sb(a){var b={},c;for(c in a)b[c]=a[c];return b}
function tb(a){if(!a||"object"!==typeof a)return a;if("function"===typeof a.clone)return a.clone();if("undefined"!==typeof Map&&a instanceof Map)return new Map(a);if("undefined"!==typeof Set&&a instanceof Set)return new Set(a);var b=Array.isArray(a)?[]:"function"!==typeof ArrayBuffer||"function"!==typeof ArrayBuffer.isView||!ArrayBuffer.isView(a)||a instanceof DataView?{}:new a.constructor(a.length),c;for(c in a)b[c]=tb(a[c]);return b}
var ub="constructor hasOwnProperty isPrototypeOf propertyIsEnumerable toLocaleString toString valueOf".split(" ");function vb(a,b){for(var c,d,e=1;e<arguments.length;e++){d=arguments[e];for(c in d)a[c]=d[c];for(var f=0;f<ub.length;f++)c=ub[f],Object.prototype.hasOwnProperty.call(d,c)&&(a[c]=d[c])}}
;var wb;function xb(){if(void 0===wb){var a=null,b=x.trustedTypes;if(b&&b.createPolicy){try{a=b.createPolicy("goog#html",{createHTML:Ya,createScript:Ya,createScriptURL:Ya})}catch(c){x.console&&x.console.error(c.message)}wb=a}else wb=a}return wb}
;function yb(a,b){this.j=a===zb&&b||""}
yb.prototype.i=!0;yb.prototype.h=function(){return this.j};
function Ab(a){return new yb(zb,a)}
var zb={};Ab("");var Bb={};function Cb(a){this.j=Bb===Bb?a:"";this.i=!0}
Cb.prototype.h=function(){return this.j.toString()};
Cb.prototype.toString=function(){return this.j.toString()};function Db(a,b){this.j=b===Eb?a:""}
Db.prototype.i=!0;Db.prototype.h=function(){return this.j.toString()};
Db.prototype.toString=function(){return this.j+""};
function Fb(a){if(a instanceof Db&&a.constructor===Db)return a.j;Ma(a);return"type_error:TrustedResourceUrl"}
var Eb={};function Gb(a){var b=xb();a=b?b.createScriptURL(a):a;return new Db(a,Eb)}
;var Hb=String.prototype.trim?function(a){return a.trim()}:function(a){return/^[\s\xa0]*([\s\S]*?)[\s\xa0]*$/.exec(a)[1]};function Ib(a,b){this.j=b===Jb?a:""}
Ib.prototype.i=!0;Ib.prototype.h=function(){return this.j.toString()};
Ib.prototype.toString=function(){return this.j.toString()};
function Kb(a){if(a instanceof Ib&&a.constructor===Ib)return a.j;Ma(a);return"type_error:SafeUrl"}
var Lb=/^(?:(?:https?|mailto|ftp):|[^:/?#]*(?:[/?#]|$))/i,Jb={};function Mb(){var a=x.navigator;return a&&(a=a.userAgent)?a:""}
function C(a){return-1!=Mb().indexOf(a)}
;function Nb(){return(C("Chrome")||C("CriOS"))&&!C("Edge")||C("Silk")}
;var Ob={};function Pb(a){this.j=Ob===Ob?a:"";this.i=!0}
Pb.prototype.h=function(){return this.j.toString()};
Pb.prototype.toString=function(){return this.j.toString()};function Qb(a,b){b instanceof Ib||b instanceof Ib||(b="object"==typeof b&&b.i?b.h():String(b),Lb.test(b)||(b="about:invalid#zClosurez"),b=new Ib(b,Jb));a.href=Kb(b)}
function Rb(a,b){a.rel="stylesheet";a.href=Fb(b).toString();(b=Sb('style[nonce],link[rel="stylesheet"][nonce]',a.ownerDocument&&a.ownerDocument.defaultView))&&a.setAttribute("nonce",b)}
function Tb(){return Sb("script[nonce]",void 0)}
var Ub=/^[\w+/_-]+[=]{0,2}$/;function Sb(a,b){b=(b||x).document;return b.querySelector?(a=b.querySelector(a))&&(a=a.nonce||a.getAttribute("nonce"))&&Ub.test(a)?a:"":""}
;function Vb(a){for(var b=0,c=0;c<a.length;++c)b=31*b+a.charCodeAt(c)>>>0;return b}
;var Wb=RegExp("^(?:([^:/?#.]+):)?(?://(?:([^\\\\/?#]*)@)?([^\\\\/?#]*?)(?::([0-9]+))?(?=[\\\\/?#]|$))?([^?#]+)?(?:\\?([^#]*))?(?:#([\\s\\S]*))?$");function Xb(a){return a?decodeURI(a):a}
function Yb(a){return Xb(a.match(Wb)[3]||null)}
function Zb(a){var b=a.match(Wb);a=b[5];var c=b[6];b=b[7];var d="";a&&(d+=a);c&&(d+="?"+c);b&&(d+="#"+b);return d}
function $b(a,b,c){if(Array.isArray(b))for(var d=0;d<b.length;d++)$b(a,String(b[d]),c);else null!=b&&c.push(a+(""===b?"":"="+encodeURIComponent(String(b))))}
function ac(a){var b=[],c;for(c in a)$b(c,a[c],b);return b.join("&")}
function bc(a,b){b=ac(b);if(b){var c=a.indexOf("#");0>c&&(c=a.length);var d=a.indexOf("?");if(0>d||d>c){d=c;var e=""}else e=a.substring(d+1,c);a=[a.substr(0,d),e,a.substr(c)];c=a[1];a[1]=b?c?c+"&"+b:b:c;b=a[0]+(a[1]?"?"+a[1]:"")+a[2]}else b=a;return b}
var cc=/#|$/;function dc(){return C("iPhone")&&!C("iPod")&&!C("iPad")}
;function ec(a){ec[" "](a);return a}
ec[" "]=Ka;var fc=C("Opera"),gc=C("Trident")||C("MSIE"),hc=C("Edge"),ic=C("Gecko")&&!(-1!=Mb().toLowerCase().indexOf("webkit")&&!C("Edge"))&&!(C("Trident")||C("MSIE"))&&!C("Edge"),jc=-1!=Mb().toLowerCase().indexOf("webkit")&&!C("Edge"),kc=C("Android");function lc(){var a=x.document;return a?a.documentMode:void 0}
var mc;a:{var nc="",oc=function(){var a=Mb();if(ic)return/rv:([^\);]+)(\)|;)/.exec(a);if(hc)return/Edge\/([\d\.]+)/.exec(a);if(gc)return/\b(?:MSIE|rv)[: ]([^\);]+)(\)|;)/.exec(a);if(jc)return/WebKit\/(\S+)/.exec(a);if(fc)return/(?:Version)[ \/]?(\S+)/.exec(a)}();
oc&&(nc=oc?oc[1]:"");if(gc){var pc=lc();if(null!=pc&&pc>parseFloat(nc)){mc=String(pc);break a}}mc=nc}var qc=mc,rc;if(x.document&&gc){var sc=lc();rc=sc?sc:parseInt(qc,10)||void 0}else rc=void 0;var tc=rc;var uc=dc()||C("iPod"),vc=C("iPad");!C("Android")||Nb();Nb();var wc=C("Safari")&&!(Nb()||C("Coast")||C("Opera")||C("Edge")||C("Edg/")||C("OPR")||C("Firefox")||C("FxiOS")||C("Silk")||C("Android"))&&!(dc()||C("iPad")||C("iPod"));var xc={},yc=null;
function zc(a,b){Na(a);void 0===b&&(b=0);if(!yc){yc={};for(var c="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789".split(""),d=["+/=","+/","-_=","-_.","-_"],e=0;5>e;e++){var f=c.concat(d[e].split(""));xc[e]=f;for(var g=0;g<f.length;g++){var h=f[g];void 0===yc[h]&&(yc[h]=g)}}}b=xc[b];c=Array(Math.floor(a.length/3));d=b[64]||"";for(e=f=0;f<a.length-2;f+=3){var k=a[f],l=a[f+1];h=a[f+2];g=b[k>>2];k=b[(k&3)<<4|l>>4];l=b[(l&15)<<2|h>>6];h=b[h&63];c[e++]=""+g+k+l+h}g=0;h=d;switch(a.length-
f){case 2:g=a[f+1],h=b[(g&15)<<2]||d;case 1:a=a[f],c[e]=""+b[a>>2]+b[(a&3)<<4|g>>4]+h+d}return c.join("")}
;var Ac="function"===typeof Uint8Array;var Bc="function"===typeof Symbol&&"symbol"===typeof Symbol()?Symbol(void 0):void 0;function Cc(a,b){Object.isFrozen(a)||(Bc?a[Bc]|=b:void 0!==a.h?a.h|=b:Object.defineProperties(a,{h:{value:b,configurable:!0,writable:!0,enumerable:!1}}))}
function Dc(a){var b;Bc?b=a[Bc]:b=a.h;return null==b?0:b}
function Ec(a){Cc(a,1);return a}
function Fc(a){return Array.isArray(a)?!!(Dc(a)&2):!1}
function Gc(a){if(!Array.isArray(a))throw Error("cannot mark non-array as immutable");Cc(a,2)}
;function Hc(a){return null!==a&&"object"===typeof a&&!Array.isArray(a)&&a.constructor===Object}
var Ic,Jc=Object.freeze(Ec([]));function Kc(a){if(Fc(a.G))throw Error("Cannot mutate an immutable Message");}
var Lc="undefined"!=typeof Symbol&&"undefined"!=typeof Symbol.hasInstance;function Mc(a,b,c){return-1===b?null:b>=a.l?a.i?a.i[b]:void 0:(void 0===c?0:c)&&a.i&&(c=a.i[b],null!=c)?c:a.G[b+a.j]}
function F(a,b,c,d,e){d=void 0===d?!1:d;(void 0===e?0:e)||Kc(a);b<a.l&&!d?a.G[b+a.j]=c:(a.i||(a.i=a.G[a.l+a.j]={}))[b]=c;return a}
function Nc(a,b,c,d){c=void 0===c?!0:c;d=void 0===d?!1:d;var e=Mc(a,b,d);null==e&&(e=Jc);if(Fc(a.G))c&&(Gc(e),Object.freeze(e));else if(e===Jc||Fc(e))e=Ec(e.slice()),F(a,b,e,d);return e}
function Oc(a,b,c,d){Kc(a);(c=Pc(a,c))&&c!==b&&null!=d&&(a.h&&c in a.h&&(a.h[c]=void 0),F(a,c,void 0));return F(a,b,d)}
function Pc(a,b){for(var c=0,d=0;d<b.length;d++){var e=b[d];null!=Mc(a,e)&&(0!==c&&F(a,c,void 0,!1,!0),c=e)}return c}
function Qc(a,b,c,d,e){if(-1===c)return null;a.h||(a.h={});var f=a.h[c];if(f)return f;e=Mc(a,c,void 0===e?!1:e);if(null==e&&!d)return f;b=new b(e);Fc(a.G)&&Gc(b.G);return a.h[c]=b}
function Rc(a,b,c,d){d=void 0===d?!1:d;a.h||(a.h={});var e=Fc(a.G),f=a.h[c];if(!f){d=Nc(a,c,!0,d);f=[];e=e||Fc(d);for(var g=0;g<d.length;g++)f[g]=new b(d[g]),e&&Gc(f[g].G);e&&(Gc(f),Object.freeze(f));a.h[c]=f}return f}
function G(a,b,c,d){d=void 0===d?!1:d;Kc(a);a.h||(a.h={});var e=c?c.G:c;a.h[b]=c;return F(a,b,e,d)}
function Sc(a,b,c){var d=Tc;Kc(a);a.h||(a.h={});var e=c?c.G:c;a.h[b]=c;Oc(a,b,d,e)}
function Uc(a,b,c,d){var e=void 0===e?!1:e;Kc(a);e=Rc(a,c,b,e);c=d?d:new c;a=Nc(a,b);e.push(c);a.push(c.G)}
;function Vc(a){switch(typeof a){case "number":return isFinite(a)?a:String(a);case "object":if(a&&!Array.isArray(a)&&Ac&&null!=a&&a instanceof Uint8Array)return zc(a)}return a}
;function Wc(a,b){b=void 0===b?Xc:b;return Yc(a,b)}
function Zc(a,b){if(null!=a){if(Array.isArray(a))a=Yc(a,b);else if(Hc(a)){var c={},d;for(d in a)c[d]=Zc(a[d],b);a=c}else a=b(a);return a}}
function Yc(a,b){for(var c=a.slice(),d=0;d<c.length;d++)c[d]=Zc(c[d],b);Array.isArray(a)&&Dc(a)&1&&Ec(c);return c}
function $c(a){if(a&&"object"==typeof a&&a.toJSON)return a.toJSON();a=Vc(a);return Array.isArray(a)?Wc(a,$c):a}
function Xc(a){return Ac&&null!=a&&a instanceof Uint8Array?new Uint8Array(a):a}
;var ad;function bd(a,b,c){var d=ad;ad=null;a||(a=d);d=this.constructor.i;a||(a=d?[d]:[]);this.j=(d?0:-1)-(this.constructor.h||0);this.h=void 0;this.G=a;a:{d=this.G.length;a=d-1;if(d&&(d=this.G[a],Hc(d))){this.l=a-this.j;this.i=d;break a}void 0!==b&&-1<b?(this.l=Math.max(b,a+1-this.j),this.i=void 0):this.l=Number.MAX_VALUE}if(c)for(b=0;b<c.length;b++)if(a=c[b],a<this.l)a+=this.j,(d=this.G[a])?Array.isArray(d)&&Ec(d):this.G[a]=Jc;else{d=this.i||(this.i=this.G[this.l+this.j]={});var e=d[a];e?Array.isArray(e)&&
Ec(e):d[a]=Jc}}
bd.prototype.toJSON=function(){var a=this.G;return Ic?a:Wc(a,$c)};
bd.prototype.clone=function(){var a=this.constructor,b=Wc(this.G);ad=b;a=new a(b);ad=null;cd(a,this);return a};
bd.prototype.toString=function(){return this.G.toString()};
function dd(a,b){return Vc(b)}
function cd(a,b){b.o&&(a.o=b.o.slice());var c=b.h;if(c){b=b.i;for(var d in c){var e=c[d];if(e){var f=!(!b||!b[d]),g=+d;if(Array.isArray(e)){if(e.length)for(f=Rc(a,e[0].constructor,g,f),g=0;g<Math.min(f.length,e.length);g++)cd(f[g],e[g])}else(f=Qc(a,e.constructor,g,void 0,f))&&cd(f,e)}}}}
;function ed(){bd.apply(this,arguments)}
r(ed,bd);function fd(){var a={};Object.defineProperties(ed,(a[Symbol.hasInstance]={value:Object[Symbol.hasInstance],configurable:!1,writable:!1,enumerable:!1},a))}
Lc&&fd();function gd(a,b){var c=this.h;if(this.isRepeated){var d=!0;d=void 0===d?!1:d;Kc(a);if(b){var e=Ec([]);for(var f=0;f<b.length;f++)e[f]=b[f].G;a.h||(a.h={});a.h[c]=b}else a.h&&(a.h[c]=void 0),e=Jc;a=F(a,c,e,d)}else a=G(a,c,b,!0);return a}
;function hd(a){x.setTimeout(function(){throw a;},0)}
;function H(){ed.apply(this,arguments)}
r(H,ed);function id(){var a={};Object.defineProperties(H,(a[Symbol.hasInstance]={value:Object[Symbol.hasInstance],configurable:!1,writable:!1,enumerable:!1},a))}
Lc&&id();function jd(){var a=this;this.promise=new Promise(function(b,c){a.resolve=b;a.reject=c})}
;function kd(a){this.i=!1;var b=a.program;a=a.globalName;var c=new jd;this.j=c.promise;this.l=q((0,x[a].a)(b,function(d,e){Promise.resolve().then(function(){c.resolve({Nb:d,vc:e})})},!0)).next().value;
this.uc=c.promise.then(function(){})}
kd.prototype.snapshot=function(a){if(this.i)throw Error("Already disposed");return this.j.then(function(b){var c=b.Nb;return new Promise(function(d){c(function(e){d(e)},[a.qb,
a.wc])})})};
kd.prototype.dispose=function(){this.i=!0;this.j.then(function(a){(a=a.vc)&&a()})};
kd.prototype.h=function(){return this.i};var ld=window;Ab("csi.gstatic.com");Ab("googleads.g.doubleclick.net");Ab("partner.googleadservices.com");Ab("pubads.g.doubleclick.net");Ab("securepubads.g.doubleclick.net");Ab("tpc.googlesyndication.com");/*

 SPDX-License-Identifier: Apache-2.0
*/
var md={};function nd(){}
function od(a){this.h=a}
r(od,nd);od.prototype.toString=function(){return this.h};
var pd=new od("about:invalid#zTSz",md);function qd(a){if(a instanceof nd)if(a instanceof od)a=a.h;else throw Error("");else a=Kb(a);return a}
;function rd(a,b){a.src=Fb(b);var c;b=(a.ownerDocument&&a.ownerDocument.defaultView||window).document;var d=null===(c=b.querySelector)||void 0===c?void 0:c.call(b,"script[nonce]");(c=d?d.nonce||d.getAttribute("nonce")||"":"")&&a.setAttribute("nonce",c)}
;function sd(a,b){this.x=void 0!==a?a:0;this.y=void 0!==b?b:0}
m=sd.prototype;m.clone=function(){return new sd(this.x,this.y)};
m.equals=function(a){return a instanceof sd&&(this==a?!0:this&&a?this.x==a.x&&this.y==a.y:!1)};
m.ceil=function(){this.x=Math.ceil(this.x);this.y=Math.ceil(this.y);return this};
m.floor=function(){this.x=Math.floor(this.x);this.y=Math.floor(this.y);return this};
m.round=function(){this.x=Math.round(this.x);this.y=Math.round(this.y);return this};
m.scale=function(a,b){this.x*=a;this.y*="number"===typeof b?b:a;return this};function td(a,b){this.width=a;this.height=b}
m=td.prototype;m.clone=function(){return new td(this.width,this.height)};
m.aspectRatio=function(){return this.width/this.height};
m.isEmpty=function(){return!(this.width*this.height)};
m.ceil=function(){this.width=Math.ceil(this.width);this.height=Math.ceil(this.height);return this};
m.floor=function(){this.width=Math.floor(this.width);this.height=Math.floor(this.height);return this};
m.round=function(){this.width=Math.round(this.width);this.height=Math.round(this.height);return this};
m.scale=function(a,b){this.width*=a;this.height*="number"===typeof b?b:a;return this};function ud(a){var b=document;return"string"===typeof a?b.getElementById(a):a}
function vd(a){var b=document;a=String(a);"application/xhtml+xml"===b.contentType&&(a=a.toLowerCase());return b.createElement(a)}
function wd(a,b){for(var c=0;a;){if(b(a))return a;a=a.parentNode;c++}return null}
;function xd(a){var b=yd;if(b)for(var c in b)Object.prototype.hasOwnProperty.call(b,c)&&a(b[c],c,b)}
function zd(){var a=[];xd(function(b){a.push(b)});
return a}
var yd={Lc:"allow-forms",Mc:"allow-modals",Nc:"allow-orientation-lock",Oc:"allow-pointer-lock",Pc:"allow-popups",Qc:"allow-popups-to-escape-sandbox",Rc:"allow-presentation",Sc:"allow-same-origin",Tc:"allow-scripts",Uc:"allow-top-navigation",Vc:"allow-top-navigation-by-user-activation"},Ad=bb(function(){return zd()});
function Bd(){var a=Dd(),b={};db(Ad(),function(c){a.sandbox&&a.sandbox.supports&&a.sandbox.supports(c)&&(b[c]=!0)});
return b}
function Dd(){var a=void 0===a?document:a;return a.createElement("iframe")}
;function Ed(a){this.isValid=a}
function Fd(a){return new Ed(function(b){return b.substr(0,a.length+1).toLowerCase()===a+":"})}
var Gd=[Fd("data"),Fd("http"),Fd("https"),Fd("mailto"),Fd("ftp"),new Ed(function(a){return/^[^:]*([/?#]|$)/.test(a)})];function Hd(a){"number"==typeof a&&(a=Math.round(a)+"px");return a}
;var Id=(new Date).getTime();function Jd(a){if(!a)return"";if(/^about:(?:blank|srcdoc)$/.test(a))return window.origin||"";a=a.split("#")[0].split("?")[0];a=a.toLowerCase();0==a.indexOf("//")&&(a=window.location.protocol+a);/^[\w\-]*:\/\//.test(a)||(a=window.location.href);var b=a.substring(a.indexOf("://")+3),c=b.indexOf("/");-1!=c&&(b=b.substring(0,c));c=a.substring(0,a.indexOf("://"));if(!c)throw Error("URI is missing protocol: "+a);if("http"!==c&&"https"!==c&&"chrome-extension"!==c&&"moz-extension"!==c&&"file"!==c&&"android-app"!==
c&&"chrome-search"!==c&&"chrome-untrusted"!==c&&"chrome"!==c&&"app"!==c&&"devtools"!==c)throw Error("Invalid URI scheme in origin: "+c);a="";var d=b.indexOf(":");if(-1!=d){var e=b.substring(d+1);b=b.substring(0,d);if("http"===c&&"80"!==e||"https"===c&&"443"!==e)a=":"+e}return c+"://"+b+a}
;function Kd(){function a(){e[0]=1732584193;e[1]=4023233417;e[2]=2562383102;e[3]=271733878;e[4]=3285377520;n=l=0}
function b(t){for(var z=g,u=0;64>u;u+=4)z[u/4]=t[u]<<24|t[u+1]<<16|t[u+2]<<8|t[u+3];for(u=16;80>u;u++)t=z[u-3]^z[u-8]^z[u-14]^z[u-16],z[u]=(t<<1|t>>>31)&4294967295;t=e[0];var D=e[1],E=e[2],L=e[3],P=e[4];for(u=0;80>u;u++){if(40>u)if(20>u){var S=L^D&(E^L);var W=1518500249}else S=D^E^L,W=1859775393;else 60>u?(S=D&E|L&(D|E),W=2400959708):(S=D^E^L,W=3395469782);S=((t<<5|t>>>27)&4294967295)+S+P+W+z[u]&4294967295;P=L;L=E;E=(D<<30|D>>>2)&4294967295;D=t;t=S}e[0]=e[0]+t&4294967295;e[1]=e[1]+D&4294967295;e[2]=
e[2]+E&4294967295;e[3]=e[3]+L&4294967295;e[4]=e[4]+P&4294967295}
function c(t,z){if("string"===typeof t){t=unescape(encodeURIComponent(t));for(var u=[],D=0,E=t.length;D<E;++D)u.push(t.charCodeAt(D));t=u}z||(z=t.length);u=0;if(0==l)for(;u+64<z;)b(t.slice(u,u+64)),u+=64,n+=64;for(;u<z;)if(f[l++]=t[u++],n++,64==l)for(l=0,b(f);u+64<z;)b(t.slice(u,u+64)),u+=64,n+=64}
function d(){var t=[],z=8*n;56>l?c(h,56-l):c(h,64-(l-56));for(var u=63;56<=u;u--)f[u]=z&255,z>>>=8;b(f);for(u=z=0;5>u;u++)for(var D=24;0<=D;D-=8)t[z++]=e[u]>>D&255;return t}
for(var e=[],f=[],g=[],h=[128],k=1;64>k;++k)h[k]=0;var l,n;a();return{reset:a,update:c,digest:d,Rb:function(){for(var t=d(),z="",u=0;u<t.length;u++)z+="0123456789ABCDEF".charAt(Math.floor(t[u]/16))+"0123456789ABCDEF".charAt(t[u]%16);return z}}}
;function Ld(a,b,c){var d=String(x.location.href);return d&&a&&b?[b,Md(Jd(d),a,c||null)].join(" "):null}
function Md(a,b,c){var d=[],e=[];if(1==(Array.isArray(c)?2:1))return e=[b,a],db(d,function(h){e.push(h)}),Nd(e.join(" "));
var f=[],g=[];db(c,function(h){g.push(h.key);f.push(h.value)});
c=Math.floor((new Date).getTime()/1E3);e=0==f.length?[c,b,a]:[f.join(":"),c,b,a];db(d,function(h){e.push(h)});
a=Nd(e.join(" "));a=[c,a];0==g.length||a.push(g.join(""));return a.join("_")}
function Nd(a){var b=Kd();b.update(a);return b.Rb().toLowerCase()}
;var Od={};function Pd(a){this.h=a||{cookie:""}}
m=Pd.prototype;m.isEnabled=function(){if(!x.navigator.cookieEnabled)return!1;if(!this.isEmpty())return!0;this.set("TESTCOOKIESENABLED","1",{Pa:60});if("1"!==this.get("TESTCOOKIESENABLED"))return!1;this.remove("TESTCOOKIESENABLED");return!0};
m.set=function(a,b,c){var d=!1;if("object"===typeof c){var e=c.Go;d=c.secure||!1;var f=c.domain||void 0;var g=c.path||void 0;var h=c.Pa}if(/[;=\s]/.test(a))throw Error('Invalid cookie name "'+a+'"');if(/[;\r\n]/.test(b))throw Error('Invalid cookie value "'+b+'"');void 0===h&&(h=-1);c=f?";domain="+f:"";g=g?";path="+g:"";d=d?";secure":"";h=0>h?"":0==h?";expires="+(new Date(1970,1,1)).toUTCString():";expires="+(new Date(Date.now()+1E3*h)).toUTCString();this.h.cookie=a+"="+b+c+g+h+d+(null!=e?";samesite="+
e:"")};
m.get=function(a,b){for(var c=a+"=",d=(this.h.cookie||"").split(";"),e=0,f;e<d.length;e++){f=Hb(d[e]);if(0==f.lastIndexOf(c,0))return f.substr(c.length);if(f==a)return""}return b};
m.remove=function(a,b,c){var d=void 0!==this.get(a);this.set(a,"",{Pa:0,path:b,domain:c});return d};
m.Za=function(){return Qd(this).keys};
m.isEmpty=function(){return!this.h.cookie};
m.clear=function(){for(var a=Qd(this).keys,b=a.length-1;0<=b;b--)this.remove(a[b])};
function Qd(a){a=(a.h.cookie||"").split(";");for(var b=[],c=[],d,e,f=0;f<a.length;f++)e=Hb(a[f]),d=e.indexOf("="),-1==d?(b.push(""),c.push(e)):(b.push(e.substring(0,d)),c.push(e.substring(d+1)));return{keys:b,values:c}}
var Rd=new Pd("undefined"==typeof document?null:document);function Sd(a){return!!Od.FPA_SAMESITE_PHASE2_MOD||!(void 0===a||!a)}
function Td(a){a=void 0===a?!1:a;var b=x.__SAPISID||x.__APISID||x.__3PSAPISID||x.__OVERRIDE_SID;Sd(a)&&(b=b||x.__1PSAPISID);if(b)return!0;var c=new Pd(document);b=c.get("SAPISID")||c.get("APISID")||c.get("__Secure-3PAPISID")||c.get("SID");Sd(a)&&(b=b||c.get("__Secure-1PAPISID"));return!!b}
function Ud(a,b,c,d){(a=x[a])||(a=(new Pd(document)).get(b));return a?Ld(a,c,d):null}
function Vd(a){var b=void 0===b?!1:b;var c=Jd(String(x.location.href)),d=[];if(Td(b)){c=0==c.indexOf("https:")||0==c.indexOf("chrome-extension:")||0==c.indexOf("moz-extension:");var e=c?x.__SAPISID:x.__APISID;e||(e=new Pd(document),e=e.get(c?"SAPISID":"APISID")||e.get("__Secure-3PAPISID"));(e=e?Ld(e,c?"SAPISIDHASH":"APISIDHASH",a):null)&&d.push(e);c&&Sd(b)&&((b=Ud("__1PSAPISID","__Secure-1PAPISID","SAPISID1PHASH",a))&&d.push(b),(a=Ud("__3PSAPISID","__Secure-3PAPISID","SAPISID3PHASH",a))&&d.push(a))}return 0==
d.length?null:d.join(" ")}
;function Wd(a){a&&"function"==typeof a.dispose&&a.dispose()}
;function Xd(a){for(var b=0,c=arguments.length;b<c;++b){var d=arguments[b];Na(d)?Xd.apply(null,d):Wd(d)}}
;function I(){this.D=this.D;this.o=this.o}
I.prototype.D=!1;I.prototype.h=function(){return this.D};
I.prototype.dispose=function(){this.D||(this.D=!0,this.I())};
function Yd(a,b){a.D?b():(a.o||(a.o=[]),a.o.push(b))}
I.prototype.I=function(){if(this.o)for(;this.o.length;)this.o.shift()()};function Zd(a,b){this.type=a;this.h=this.target=b;this.defaultPrevented=this.j=!1}
Zd.prototype.stopPropagation=function(){this.j=!0};
Zd.prototype.preventDefault=function(){this.defaultPrevented=!0};function $d(a){var b=A("window.location.href");null==a&&(a='Unknown Error of type "null/undefined"');if("string"===typeof a)return{message:a,name:"Unknown error",lineNumber:"Not available",fileName:b,stack:"Not available"};var c=!1;try{var d=a.lineNumber||a.line||"Not available"}catch(g){d="Not available",c=!0}try{var e=a.fileName||a.filename||a.sourceURL||x.$googDebugFname||b}catch(g){e="Not available",c=!0}b=ae(a);if(!(!c&&a.lineNumber&&a.fileName&&a.stack&&a.message&&a.name)){c=a.message;if(null==
c){if(a.constructor&&a.constructor instanceof Function){if(a.constructor.name)c=a.constructor.name;else if(c=a.constructor,be[c])c=be[c];else{c=String(c);if(!be[c]){var f=/function\s+([^\(]+)/m.exec(c);be[c]=f?f[1]:"[Anonymous]"}c=be[c]}c='Unknown Error of type "'+c+'"'}else c="Unknown Error of unknown type";"function"===typeof a.toString&&Object.prototype.toString!==a.toString&&(c+=": "+a.toString())}return{message:c,name:a.name||"UnknownError",lineNumber:d,fileName:e,stack:b||"Not available"}}a.stack=
b;return{message:a.message,name:a.name,lineNumber:a.lineNumber,fileName:a.fileName,stack:a.stack}}
function ae(a,b){b||(b={});b[ce(a)]=!0;var c=a.stack||"";(a=a.Ob)&&!b[ce(a)]&&(c+="\nCaused by: ",a.stack&&0==a.stack.indexOf(a.toString())||(c+="string"===typeof a?a:a.message+"\n"),c+=ae(a,b));return c}
function ce(a){var b="";"function"===typeof a.toString&&(b=""+a);return b+a.stack}
var be={};var de=function(){if(!x.addEventListener||!Object.defineProperty)return!1;var a=!1,b=Object.defineProperty({},"passive",{get:function(){a=!0}});
try{x.addEventListener("test",Ka,b),x.removeEventListener("test",Ka,b)}catch(c){}return a}();function ee(a,b){Zd.call(this,a?a.type:"");this.relatedTarget=this.h=this.target=null;this.button=this.screenY=this.screenX=this.clientY=this.clientX=0;this.key="";this.charCode=this.keyCode=0;this.metaKey=this.shiftKey=this.altKey=this.ctrlKey=!1;this.state=null;this.pointerId=0;this.pointerType="";this.i=null;a&&this.init(a,b)}
Xa(ee,Zd);var fe={2:"touch",3:"pen",4:"mouse"};
ee.prototype.init=function(a,b){var c=this.type=a.type,d=a.changedTouches&&a.changedTouches.length?a.changedTouches[0]:null;this.target=a.target||a.srcElement;this.h=b;if(b=a.relatedTarget){if(ic){a:{try{ec(b.nodeName);var e=!0;break a}catch(f){}e=!1}e||(b=null)}}else"mouseover"==c?b=a.fromElement:"mouseout"==c&&(b=a.toElement);this.relatedTarget=b;d?(this.clientX=void 0!==d.clientX?d.clientX:d.pageX,this.clientY=void 0!==d.clientY?d.clientY:d.pageY,this.screenX=d.screenX||0,this.screenY=d.screenY||
0):(this.clientX=void 0!==a.clientX?a.clientX:a.pageX,this.clientY=void 0!==a.clientY?a.clientY:a.pageY,this.screenX=a.screenX||0,this.screenY=a.screenY||0);this.button=a.button;this.keyCode=a.keyCode||0;this.key=a.key||"";this.charCode=a.charCode||("keypress"==c?a.keyCode:0);this.ctrlKey=a.ctrlKey;this.altKey=a.altKey;this.shiftKey=a.shiftKey;this.metaKey=a.metaKey;this.pointerId=a.pointerId||0;this.pointerType="string"===typeof a.pointerType?a.pointerType:fe[a.pointerType]||"";this.state=a.state;
this.i=a;a.defaultPrevented&&ee.aa.preventDefault.call(this)};
ee.prototype.stopPropagation=function(){ee.aa.stopPropagation.call(this);this.i.stopPropagation?this.i.stopPropagation():this.i.cancelBubble=!0};
ee.prototype.preventDefault=function(){ee.aa.preventDefault.call(this);var a=this.i;a.preventDefault?a.preventDefault():a.returnValue=!1};var ge="closure_listenable_"+(1E6*Math.random()|0);var he=0;function ie(a,b,c,d,e){this.listener=a;this.proxy=null;this.src=b;this.type=c;this.capture=!!d;this.Ma=e;this.key=++he;this.za=this.Ia=!1}
function je(a){a.za=!0;a.listener=null;a.proxy=null;a.src=null;a.Ma=null}
;function ke(a){this.src=a;this.listeners={};this.h=0}
ke.prototype.add=function(a,b,c,d,e){var f=a.toString();a=this.listeners[f];a||(a=this.listeners[f]=[],this.h++);var g=le(a,b,d,e);-1<g?(b=a[g],c||(b.Ia=!1)):(b=new ie(b,this.src,f,!!d,e),b.Ia=c,a.push(b));return b};
ke.prototype.remove=function(a,b,c,d){a=a.toString();if(!(a in this.listeners))return!1;var e=this.listeners[a];b=le(e,b,c,d);return-1<b?(je(e[b]),Array.prototype.splice.call(e,b,1),0==e.length&&(delete this.listeners[a],this.h--),!0):!1};
function me(a,b){var c=b.type;c in a.listeners&&ib(a.listeners[c],b)&&(je(b),0==a.listeners[c].length&&(delete a.listeners[c],a.h--))}
function le(a,b,c,d){for(var e=0;e<a.length;++e){var f=a[e];if(!f.za&&f.listener==b&&f.capture==!!c&&f.Ma==d)return e}return-1}
;var ne="closure_lm_"+(1E6*Math.random()|0),oe={},pe=0;function qe(a,b,c,d,e){if(d&&d.once)re(a,b,c,d,e);else if(Array.isArray(b))for(var f=0;f<b.length;f++)qe(a,b[f],c,d,e);else c=se(c),a&&a[ge]?a.W(b,c,Oa(d)?!!d.capture:!!d,e):te(a,b,c,!1,d,e)}
function te(a,b,c,d,e,f){if(!b)throw Error("Invalid event type");var g=Oa(e)?!!e.capture:!!e,h=ue(a);h||(a[ne]=h=new ke(a));c=h.add(b,c,d,g,f);if(!c.proxy){d=ve();c.proxy=d;d.src=a;d.listener=c;if(a.addEventListener)de||(e=g),void 0===e&&(e=!1),a.addEventListener(b.toString(),d,e);else if(a.attachEvent)a.attachEvent(we(b.toString()),d);else if(a.addListener&&a.removeListener)a.addListener(d);else throw Error("addEventListener and attachEvent are unavailable.");pe++}}
function ve(){function a(c){return b.call(a.src,a.listener,c)}
var b=xe;return a}
function re(a,b,c,d,e){if(Array.isArray(b))for(var f=0;f<b.length;f++)re(a,b[f],c,d,e);else c=se(c),a&&a[ge]?a.i.add(String(b),c,!0,Oa(d)?!!d.capture:!!d,e):te(a,b,c,!0,d,e)}
function ye(a,b,c,d,e){if(Array.isArray(b))for(var f=0;f<b.length;f++)ye(a,b[f],c,d,e);else(d=Oa(d)?!!d.capture:!!d,c=se(c),a&&a[ge])?a.i.remove(String(b),c,d,e):a&&(a=ue(a))&&(b=a.listeners[b.toString()],a=-1,b&&(a=le(b,c,d,e)),(c=-1<a?b[a]:null)&&ze(c))}
function ze(a){if("number"!==typeof a&&a&&!a.za){var b=a.src;if(b&&b[ge])me(b.i,a);else{var c=a.type,d=a.proxy;b.removeEventListener?b.removeEventListener(c,d,a.capture):b.detachEvent?b.detachEvent(we(c),d):b.addListener&&b.removeListener&&b.removeListener(d);pe--;(c=ue(b))?(me(c,a),0==c.h&&(c.src=null,b[ne]=null)):je(a)}}}
function we(a){return a in oe?oe[a]:oe[a]="on"+a}
function xe(a,b){if(a.za)a=!0;else{b=new ee(b,this);var c=a.listener,d=a.Ma||a.src;a.Ia&&ze(a);a=c.call(d,b)}return a}
function ue(a){a=a[ne];return a instanceof ke?a:null}
var Ae="__closure_events_fn_"+(1E9*Math.random()>>>0);function se(a){if("function"===typeof a)return a;a[Ae]||(a[Ae]=function(b){return a.handleEvent(b)});
return a[Ae]}
;function Be(){I.call(this);this.i=new ke(this);this.Z=this;this.L=null}
Xa(Be,I);Be.prototype[ge]=!0;Be.prototype.addEventListener=function(a,b,c,d){qe(this,a,b,c,d)};
Be.prototype.removeEventListener=function(a,b,c,d){ye(this,a,b,c,d)};
function Ce(a,b){var c=a.L;if(c){var d=[];for(var e=1;c;c=c.L)d.push(c),++e}a=a.Z;c=b.type||b;"string"===typeof b?b=new Zd(b,a):b instanceof Zd?b.target=b.target||a:(e=b,b=new Zd(c,a),vb(b,e));e=!0;if(d)for(var f=d.length-1;!b.j&&0<=f;f--){var g=b.h=d[f];e=De(g,c,!0,b)&&e}b.j||(g=b.h=a,e=De(g,c,!0,b)&&e,b.j||(e=De(g,c,!1,b)&&e));if(d)for(f=0;!b.j&&f<d.length;f++)g=b.h=d[f],e=De(g,c,!1,b)&&e}
Be.prototype.I=function(){Be.aa.I.call(this);if(this.i){var a=this.i,b=0,c;for(c in a.listeners){for(var d=a.listeners[c],e=0;e<d.length;e++)++b,je(d[e]);delete a.listeners[c];a.h--}}this.L=null};
Be.prototype.W=function(a,b,c,d){return this.i.add(String(a),b,!1,c,d)};
function De(a,b,c,d){b=a.i.listeners[String(b)];if(!b)return!0;b=b.concat();for(var e=!0,f=0;f<b.length;++f){var g=b[f];if(g&&!g.za&&g.capture==c){var h=g.listener,k=g.Ma||g.src;g.Ia&&me(a.i,g);e=!1!==h.call(k,d)&&e}}return e&&!d.defaultPrevented}
;function Ee(a){var b,c;Be.call(this);var d=this;this.A=this.l=0;this.V=null!==a&&void 0!==a?a:{N:function(e,f){return setTimeout(e,f)},
U:clearTimeout};this.j=null!==(c=null===(b=window.navigator)||void 0===b?void 0:b.onLine)&&void 0!==c?c:!0;this.m=function(){return w(function(e){return v(e,Fe(d),0)})};
window.addEventListener("offline",this.m);window.addEventListener("online",this.m);this.A||Ge(this)}
r(Ee,Be);Ee.prototype.dispose=function(){window.removeEventListener("offline",this.m);window.removeEventListener("online",this.m);this.V.U(this.A);delete Ee.h};
Ee.prototype.H=function(){return this.j};
function Ge(a){a.A=a.V.N(function(){var b;return w(function(c){if(1==c.h)return a.j?(null===(b=window.navigator)||void 0===b?0:b.onLine)?c.s(3):v(c,Fe(a),3):v(c,Fe(a),3);Ge(a);c.h=0})},3E4)}
function Fe(a,b){return a.u?a.u:a.u=new Promise(function(c){var d,e,f;return w(function(g){switch(g.h){case 1:return d=window.AbortController?new window.AbortController:void 0,e=null===d||void 0===d?void 0:d.signal,f=!1,ta(g,2,3),d&&(a.l=a.V.N(function(){d.abort()},b||2E4)),v(g,fetch("/generate_204",{method:"HEAD",
signal:e}),5);case 5:f=!0;case 3:xa(g);a.u=void 0;a.l&&(a.V.U(a.l),a.l=0);f!==a.j&&(a.j=f,a.j?Ce(a,"networkstatus-online"):Ce(a,"networkstatus-offline"));c(f);ya(g);break;case 2:wa(g),f=!1,g.s(3)}})})}
;var He={bo:"WEB_DISPLAY_MODE_UNKNOWN",Xn:"WEB_DISPLAY_MODE_BROWSER",Zn:"WEB_DISPLAY_MODE_MINIMAL_UI",ao:"WEB_DISPLAY_MODE_STANDALONE",Yn:"WEB_DISPLAY_MODE_FULLSCREEN"};function Ie(){this.data_=[];this.h=-1}
Ie.prototype.set=function(a,b){b=void 0===b?!0:b;0<=a&&52>a&&0===a%1&&this.data_[a]!=b&&(this.data_[a]=b,this.h=-1)};
Ie.prototype.get=function(a){return!!this.data_[a]};
function Je(a){-1==a.h&&(a.h=gb(a.data_,function(b,c,d){return c?b+Math.pow(2,d):b},0));
return a.h}
;function Ke(a,b){this.j=a;this.l=b;this.i=0;this.h=null}
Ke.prototype.get=function(){if(0<this.i){this.i--;var a=this.h;this.h=a.next;a.next=null}else a=this.j();return a};
function Le(a,b){a.l(b);100>a.i&&(a.i++,b.next=a.h,a.h=b)}
;var Me;function Ne(){var a=x.MessageChannel;"undefined"===typeof a&&"undefined"!==typeof window&&window.postMessage&&window.addEventListener&&!C("Presto")&&(a=function(){var e=vd("IFRAME");e.style.display="none";document.documentElement.appendChild(e);var f=e.contentWindow;e=f.document;e.open();e.close();var g="callImmediate"+Math.random(),h="file:"==f.location.protocol?"*":f.location.protocol+"//"+f.location.host;e=Ua(function(k){if(("*"==h||k.origin==h)&&k.data==g)this.port1.onmessage()},this);
f.addEventListener("message",e,!1);this.port1={};this.port2={postMessage:function(){f.postMessage(g,h)}}});
if("undefined"!==typeof a&&!C("Trident")&&!C("MSIE")){var b=new a,c={},d=c;b.port1.onmessage=function(){if(void 0!==c.next){c=c.next;var e=c.ob;c.ob=null;e()}};
return function(e){d.next={ob:e};d=d.next;b.port2.postMessage(0)}}return function(e){x.setTimeout(e,0)}}
;function Oe(){this.i=this.h=null}
Oe.prototype.add=function(a,b){var c=Pe.get();c.set(a,b);this.i?this.i.next=c:this.h=c;this.i=c};
Oe.prototype.remove=function(){var a=null;this.h&&(a=this.h,this.h=this.h.next,this.h||(this.i=null),a.next=null);return a};
var Pe=new Ke(function(){return new Qe},function(a){return a.reset()});
function Qe(){this.next=this.scope=this.h=null}
Qe.prototype.set=function(a,b){this.h=a;this.scope=b;this.next=null};
Qe.prototype.reset=function(){this.next=this.scope=this.h=null};function Re(a,b){Se||Te();Ue||(Se(),Ue=!0);Ve.add(a,b)}
var Se;function Te(){if(x.Promise&&x.Promise.resolve){var a=x.Promise.resolve(void 0);Se=function(){a.then(We)}}else Se=function(){var b=We;
"function"!==typeof x.setImmediate||x.Window&&x.Window.prototype&&!C("Edge")&&x.Window.prototype.setImmediate==x.setImmediate?(Me||(Me=Ne()),Me(b)):x.setImmediate(b)}}
var Ue=!1,Ve=new Oe;function We(){for(var a;a=Ve.remove();){try{a.h.call(a.scope)}catch(b){hd(b)}Le(Pe,a)}Ue=!1}
;function Xe(a,b){this.h=a[x.Symbol.iterator]();this.i=b;this.j=0}
Xe.prototype[Symbol.iterator]=function(){return this};
Xe.prototype.next=function(){var a=this.h.next();return{value:a.done?void 0:this.i.call(void 0,a.value,this.j++),done:a.done}};
function Ye(a,b){return new Xe(a,b)}
;function Ze(){this.blockSize=-1}
;function $e(){this.blockSize=-1;this.blockSize=64;this.h=[];this.o=[];this.m=[];this.j=[];this.j[0]=128;for(var a=1;a<this.blockSize;++a)this.j[a]=0;this.l=this.i=0;this.reset()}
Xa($e,Ze);$e.prototype.reset=function(){this.h[0]=1732584193;this.h[1]=4023233417;this.h[2]=2562383102;this.h[3]=271733878;this.h[4]=3285377520;this.l=this.i=0};
function af(a,b,c){c||(c=0);var d=a.m;if("string"===typeof b)for(var e=0;16>e;e++)d[e]=b.charCodeAt(c)<<24|b.charCodeAt(c+1)<<16|b.charCodeAt(c+2)<<8|b.charCodeAt(c+3),c+=4;else for(e=0;16>e;e++)d[e]=b[c]<<24|b[c+1]<<16|b[c+2]<<8|b[c+3],c+=4;for(e=16;80>e;e++){var f=d[e-3]^d[e-8]^d[e-14]^d[e-16];d[e]=(f<<1|f>>>31)&4294967295}b=a.h[0];c=a.h[1];var g=a.h[2],h=a.h[3],k=a.h[4];for(e=0;80>e;e++){if(40>e)if(20>e){f=h^c&(g^h);var l=1518500249}else f=c^g^h,l=1859775393;else 60>e?(f=c&g|h&(c|g),l=2400959708):
(f=c^g^h,l=3395469782);f=(b<<5|b>>>27)+f+k+l+d[e]&4294967295;k=h;h=g;g=(c<<30|c>>>2)&4294967295;c=b;b=f}a.h[0]=a.h[0]+b&4294967295;a.h[1]=a.h[1]+c&4294967295;a.h[2]=a.h[2]+g&4294967295;a.h[3]=a.h[3]+h&4294967295;a.h[4]=a.h[4]+k&4294967295}
$e.prototype.update=function(a,b){if(null!=a){void 0===b&&(b=a.length);for(var c=b-this.blockSize,d=0,e=this.o,f=this.i;d<b;){if(0==f)for(;d<=c;)af(this,a,d),d+=this.blockSize;if("string"===typeof a)for(;d<b;){if(e[f]=a.charCodeAt(d),++f,++d,f==this.blockSize){af(this,e);f=0;break}}else for(;d<b;)if(e[f]=a[d],++f,++d,f==this.blockSize){af(this,e);f=0;break}}this.i=f;this.l+=b}};
$e.prototype.digest=function(){var a=[],b=8*this.l;56>this.i?this.update(this.j,56-this.i):this.update(this.j,this.blockSize-(this.i-56));for(var c=this.blockSize-1;56<=c;c--)this.o[c]=b&255,b/=256;af(this,this.o);for(c=b=0;5>c;c++)for(var d=24;0<=d;d-=8)a[b]=this.h[c]>>d&255,++b;return a};function bf(a){return"string"==typeof a.className?a.className:a.getAttribute&&a.getAttribute("class")||""}
function cf(a,b){"string"==typeof a.className?a.className=b:a.setAttribute&&a.setAttribute("class",b)}
function df(a,b){a.classList?b=a.classList.contains(b):(a=a.classList?a.classList:bf(a).match(/\S+/g)||[],b=0<=cb(a,b));return b}
function ef(){var a=document.body;a.classList?a.classList.remove("inverted-hdpi"):df(a,"inverted-hdpi")&&cf(a,Array.prototype.filter.call(a.classList?a.classList:bf(a).match(/\S+/g)||[],function(b){return"inverted-hdpi"!=b}).join(" "))}
;var ff="StopIteration"in x?x.StopIteration:{message:"StopIteration",stack:""};function gf(){}
gf.prototype.da=function(){throw ff;};
gf.prototype.next=function(){return hf};
var hf={done:!0,value:void 0};function jf(a){return{value:a,done:!1}}
function kf(a){if(a.done)throw ff;return a.value}
gf.prototype.T=function(){return this};function lf(a){if(a instanceof mf||a instanceof nf||a instanceof of)return a;if("function"==typeof a.da)return new mf(function(){return pf(a)});
if("function"==typeof a[Symbol.iterator])return new mf(function(){return a[Symbol.iterator]()});
if("function"==typeof a.T)return new mf(function(){return pf(a.T())});
throw Error("Not an iterator or iterable.");}
function pf(a){if(!(a instanceof gf))return a;var b=!1;return{next:function(){for(var c;!b;)try{c=a.da();break}catch(d){if(d!==ff)throw d;b=!0}return{value:c,done:b}}}}
function mf(a){this.h=a}
mf.prototype.T=function(){return new nf(this.h())};
mf.prototype[Symbol.iterator]=function(){return new of(this.h())};
mf.prototype.i=function(){return new of(this.h())};
function nf(a){this.h=a}
r(nf,gf);nf.prototype.da=function(){var a=this.h.next();if(a.done)throw ff;return a.value};
nf.prototype.next=function(){return this.h.next()};
nf.prototype[Symbol.iterator]=function(){return new of(this.h)};
nf.prototype.i=function(){return new of(this.h)};
function of(a){mf.call(this,function(){return a});
this.j=a}
r(of,mf);of.prototype.next=function(){return this.j.next()};function qf(a,b){this.i={};this.h=[];this.ja=this.size=0;var c=arguments.length;if(1<c){if(c%2)throw Error("Uneven number of arguments");for(var d=0;d<c;d+=2)this.set(arguments[d],arguments[d+1])}else if(a)if(a instanceof qf)for(c=a.Za(),d=0;d<c.length;d++)this.set(c[d],a.get(c[d]));else for(d in a)this.set(d,a[d])}
m=qf.prototype;m.Za=function(){rf(this);return this.h.concat()};
m.has=function(a){return sf(this.i,a)};
m.equals=function(a,b){if(this===a)return!0;if(this.size!=a.size)return!1;b=b||tf;rf(this);for(var c,d=0;c=this.h[d];d++)if(!b(this.get(c),a.get(c)))return!1;return!0};
function tf(a,b){return a===b}
m.isEmpty=function(){return 0==this.size};
m.clear=function(){this.i={};this.ja=this.size=this.h.length=0};
m.remove=function(a){return this.delete(a)};
m.delete=function(a){return sf(this.i,a)?(delete this.i[a],--this.size,this.ja++,this.h.length>2*this.size&&rf(this),!0):!1};
function rf(a){if(a.size!=a.h.length){for(var b=0,c=0;b<a.h.length;){var d=a.h[b];sf(a.i,d)&&(a.h[c++]=d);b++}a.h.length=c}if(a.size!=a.h.length){var e={};for(c=b=0;b<a.h.length;)d=a.h[b],sf(e,d)||(a.h[c++]=d,e[d]=1),b++;a.h.length=c}}
m.get=function(a,b){return sf(this.i,a)?this.i[a]:b};
m.set=function(a,b){sf(this.i,a)||(this.size+=1,this.h.push(a),this.ja++);this.i[a]=b};
m.forEach=function(a,b){for(var c=this.Za(),d=0;d<c.length;d++){var e=c[d],f=this.get(e);a.call(b,f,e,this)}};
m.clone=function(){return new qf(this)};
m.keys=function(){return lf(this.T(!0)).i()};
m.values=function(){return lf(this.T(!1)).i()};
m.entries=function(){var a=this;return Ye(this.keys(),function(b){return[b,a.get(b)]})};
m.T=function(a){rf(this);var b=0,c=this.ja,d=this,e=new gf;e.next=function(){if(c!=d.ja)throw Error("The map has changed since the iterator was created");if(b>=d.h.length)return hf;var g=d.h[b++];return jf(a?g:d.i[g])};
var f=e.next;e.da=function(){return kf(f.call(e))};
return e};
function sf(a,b){return Object.prototype.hasOwnProperty.call(a,b)}
;function uf(a){vf();return Gb(a)}
var vf=Ka;function wf(a){var b=[];xf(new yf,a,b);return b.join("")}
function yf(){}
function xf(a,b,c){if(null==b)c.push("null");else{if("object"==typeof b){if(Array.isArray(b)){var d=b;b=d.length;c.push("[");for(var e="",f=0;f<b;f++)c.push(e),xf(a,d[f],c),e=",";c.push("]");return}if(b instanceof String||b instanceof Number||b instanceof Boolean)b=b.valueOf();else{c.push("{");e="";for(d in b)Object.prototype.hasOwnProperty.call(b,d)&&(f=b[d],"function"!=typeof f&&(c.push(e),zf(d,c),c.push(":"),xf(a,f,c),e=","));c.push("}");return}}switch(typeof b){case "string":zf(b,c);break;case "number":c.push(isFinite(b)&&
!isNaN(b)?String(b):"null");break;case "boolean":c.push(String(b));break;case "function":c.push("null");break;default:throw Error("Unknown type: "+typeof b);}}}
var Af={'"':'\\"',"\\":"\\\\","/":"\\/","\b":"\\b","\f":"\\f","\n":"\\n","\r":"\\r","\t":"\\t","\v":"\\u000b"},Bf=/\uffff/.test("\uffff")?/[\\"\x00-\x1f\x7f-\uffff]/g:/[\\"\x00-\x1f\x7f-\xff]/g;function zf(a,b){b.push('"',a.replace(Bf,function(c){var d=Af[c];d||(d="\\u"+(c.charCodeAt(0)|65536).toString(16).substr(1),Af[c]=d);return d}),'"')}
;function Cf(a){if(!a)return!1;try{return!!a.$goog_Thenable}catch(b){return!1}}
;function Df(a){this.h=0;this.D=void 0;this.l=this.i=this.j=null;this.o=this.m=!1;if(a!=Ka)try{var b=this;a.call(void 0,function(c){Ef(b,2,c)},function(c){Ef(b,3,c)})}catch(c){Ef(this,3,c)}}
function Ff(){this.next=this.context=this.onRejected=this.i=this.h=null;this.j=!1}
Ff.prototype.reset=function(){this.context=this.onRejected=this.i=this.h=null;this.j=!1};
var Gf=new Ke(function(){return new Ff},function(a){a.reset()});
function Hf(a,b,c){var d=Gf.get();d.i=a;d.onRejected=b;d.context=c;return d}
function If(a){return new Df(function(b,c){c(a)})}
Df.prototype.then=function(a,b,c){return Jf(this,"function"===typeof a?a:null,"function"===typeof b?b:null,c)};
Df.prototype.$goog_Thenable=!0;function Kf(a,b){return Jf(a,null,b,void 0)}
Df.prototype.cancel=function(a){if(0==this.h){var b=new Lf(a);Re(function(){Mf(this,b)},this)}};
function Mf(a,b){if(0==a.h)if(a.j){var c=a.j;if(c.i){for(var d=0,e=null,f=null,g=c.i;g&&(g.j||(d++,g.h==a&&(e=g),!(e&&1<d)));g=g.next)e||(f=g);e&&(0==c.h&&1==d?Mf(c,b):(f?(d=f,d.next==c.l&&(c.l=d),d.next=d.next.next):Nf(c),Of(c,e,3,b)))}a.j=null}else Ef(a,3,b)}
function Pf(a,b){a.i||2!=a.h&&3!=a.h||Qf(a);a.l?a.l.next=b:a.i=b;a.l=b}
function Jf(a,b,c,d){var e=Hf(null,null,null);e.h=new Df(function(f,g){e.i=b?function(h){try{var k=b.call(d,h);f(k)}catch(l){g(l)}}:f;
e.onRejected=c?function(h){try{var k=c.call(d,h);void 0===k&&h instanceof Lf?g(h):f(k)}catch(l){g(l)}}:g});
e.h.j=a;Pf(a,e);return e.h}
Df.prototype.A=function(a){this.h=0;Ef(this,2,a)};
Df.prototype.L=function(a){this.h=0;Ef(this,3,a)};
function Ef(a,b,c){if(0==a.h){a===c&&(b=3,c=new TypeError("Promise cannot resolve to itself"));a.h=1;a:{var d=c,e=a.A,f=a.L;if(d instanceof Df){Pf(d,Hf(e||Ka,f||null,a));var g=!0}else if(Cf(d))d.then(e,f,a),g=!0;else{if(Oa(d))try{var h=d.then;if("function"===typeof h){Rf(d,h,e,f,a);g=!0;break a}}catch(k){f.call(a,k);g=!0;break a}g=!1}}g||(a.D=c,a.h=b,a.j=null,Qf(a),3!=b||c instanceof Lf||Sf(a,c))}}
function Rf(a,b,c,d,e){function f(k){h||(h=!0,d.call(e,k))}
function g(k){h||(h=!0,c.call(e,k))}
var h=!1;try{b.call(a,g,f)}catch(k){f(k)}}
function Qf(a){a.m||(a.m=!0,Re(a.u,a))}
function Nf(a){var b=null;a.i&&(b=a.i,a.i=b.next,b.next=null);a.i||(a.l=null);return b}
Df.prototype.u=function(){for(var a;a=Nf(this);)Of(this,a,this.h,this.D);this.m=!1};
function Of(a,b,c,d){if(3==c&&b.onRejected&&!b.j)for(;a&&a.o;a=a.j)a.o=!1;if(b.h)b.h.j=null,Tf(b,c,d);else try{b.j?b.i.call(b.context):Tf(b,c,d)}catch(e){Uf.call(null,e)}Le(Gf,b)}
function Tf(a,b,c){2==b?a.i.call(a.context,c):a.onRejected&&a.onRejected.call(a.context,c)}
function Sf(a,b){a.o=!0;Re(function(){a.o&&Uf.call(null,b)})}
var Uf=hd;function Lf(a){$a.call(this,a)}
Xa(Lf,$a);Lf.prototype.name="cancel";function J(a){I.call(this);this.u=1;this.l=[];this.m=0;this.i=[];this.j={};this.A=!!a}
Xa(J,I);m=J.prototype;m.subscribe=function(a,b,c){var d=this.j[a];d||(d=this.j[a]=[]);var e=this.u;this.i[e]=a;this.i[e+1]=b;this.i[e+2]=c;this.u=e+3;d.push(e);return e};
function Vf(a,b,c,d){if(b=a.j[b]){var e=a.i;(b=b.find(function(f){return e[f+1]==c&&e[f+2]==d}))&&a.va(b)}}
m.va=function(a){var b=this.i[a];if(b){var c=this.j[b];0!=this.m?(this.l.push(a),this.i[a+1]=Ka):(c&&ib(c,a),delete this.i[a],delete this.i[a+1],delete this.i[a+2])}return!!b};
m.ka=function(a,b){var c=this.j[a];if(c){for(var d=Array(arguments.length-1),e=1,f=arguments.length;e<f;e++)d[e-1]=arguments[e];if(this.A)for(e=0;e<c.length;e++){var g=c[e];Wf(this.i[g+1],this.i[g+2],d)}else{this.m++;try{for(e=0,f=c.length;e<f&&!this.h();e++)g=c[e],this.i[g+1].apply(this.i[g+2],d)}finally{if(this.m--,0<this.l.length&&0==this.m)for(;c=this.l.pop();)this.va(c)}}return 0!=e}return!1};
function Wf(a,b,c){Re(function(){a.apply(b,c)})}
m.clear=function(a){if(a){var b=this.j[a];b&&(b.forEach(this.va,this),delete this.j[a])}else this.i.length=0,this.j={}};
m.I=function(){J.aa.I.call(this);this.clear();this.l.length=0};function Xf(a){this.h=a}
Xf.prototype.set=function(a,b){void 0===b?this.h.remove(a):this.h.set(a,wf(b))};
Xf.prototype.get=function(a){try{var b=this.h.get(a)}catch(c){return}if(null!==b)try{return JSON.parse(b)}catch(c){throw"Storage: Invalid value was encountered";}};
Xf.prototype.remove=function(a){this.h.remove(a)};function Yf(a){this.h=a}
Xa(Yf,Xf);function Zf(a){this.data=a}
function $f(a){return void 0===a||a instanceof Zf?a:new Zf(a)}
Yf.prototype.set=function(a,b){Yf.aa.set.call(this,a,$f(b))};
Yf.prototype.i=function(a){a=Yf.aa.get.call(this,a);if(void 0===a||a instanceof Object)return a;throw"Storage: Invalid value was encountered";};
Yf.prototype.get=function(a){if(a=this.i(a)){if(a=a.data,void 0===a)throw"Storage: Invalid value was encountered";}else a=void 0;return a};function ag(a){this.h=a}
Xa(ag,Yf);ag.prototype.set=function(a,b,c){if(b=$f(b)){if(c){if(c<Date.now()){ag.prototype.remove.call(this,a);return}b.expiration=c}b.creation=Date.now()}ag.aa.set.call(this,a,b)};
ag.prototype.i=function(a){var b=ag.aa.i.call(this,a);if(b){var c=b.creation,d=b.expiration;if(d&&d<Date.now()||c&&c>Date.now())ag.prototype.remove.call(this,a);else return b}};function bg(){}
;function cg(){}
Xa(cg,bg);cg.prototype[Symbol.iterator]=function(){return lf(this.T(!0)).i()};
cg.prototype.clear=function(){var a=Array.from(this);a=q(a);for(var b=a.next();!b.done;b=a.next())this.remove(b.value)};function dg(a){this.h=a}
Xa(dg,cg);m=dg.prototype;m.isAvailable=function(){if(!this.h)return!1;try{return this.h.setItem("__sak","1"),this.h.removeItem("__sak"),!0}catch(a){return!1}};
m.set=function(a,b){try{this.h.setItem(a,b)}catch(c){if(0==this.h.length)throw"Storage mechanism: Storage disabled";throw"Storage mechanism: Quota exceeded";}};
m.get=function(a){a=this.h.getItem(a);if("string"!==typeof a&&null!==a)throw"Storage mechanism: Invalid value was encountered";return a};
m.remove=function(a){this.h.removeItem(a)};
m.T=function(a){var b=0,c=this.h,d=new gf;d.next=function(){if(b>=c.length)return hf;var f=c.key(b++);if(a)return jf(f);f=c.getItem(f);if("string"!==typeof f)throw"Storage mechanism: Invalid value was encountered";return jf(f)};
var e=d.next;d.da=function(){return kf(e.call(d))};
return d};
m.clear=function(){this.h.clear()};
m.key=function(a){return this.h.key(a)};function eg(){var a=null;try{a=window.localStorage||null}catch(b){}this.h=a}
Xa(eg,dg);function fg(a,b){this.i=a;this.h=null;var c;if(c=gc)c=!(9<=Number(tc));if(c){gg||(gg=new qf);this.h=gg.get(a);this.h||(b?this.h=document.getElementById(b):(this.h=document.createElement("userdata"),this.h.addBehavior("#default#userData"),document.body.appendChild(this.h)),gg.set(a,this.h));try{this.h.load(this.i)}catch(d){this.h=null}}}
Xa(fg,cg);var hg={".":".2E","!":".21","~":".7E","*":".2A","'":".27","(":".28",")":".29","%":"."},gg=null;function ig(a){return"_"+encodeURIComponent(a).replace(/[.!~*'()%]/g,function(b){return hg[b]})}
m=fg.prototype;m.isAvailable=function(){return!!this.h};
m.set=function(a,b){this.h.setAttribute(ig(a),b);jg(this)};
m.get=function(a){a=this.h.getAttribute(ig(a));if("string"!==typeof a&&null!==a)throw"Storage mechanism: Invalid value was encountered";return a};
m.remove=function(a){this.h.removeAttribute(ig(a));jg(this)};
m.T=function(a){var b=0,c=this.h.XMLDocument.documentElement.attributes,d=new gf;d.next=function(){if(b>=c.length)return hf;var f=c[b++];if(a)return jf(decodeURIComponent(f.nodeName.replace(/\./g,"%")).substr(1));f=f.nodeValue;if("string"!==typeof f)throw"Storage mechanism: Invalid value was encountered";return jf(f)};
var e=d.next;d.da=function(){return kf(e.call(d))};
return d};
m.clear=function(){for(var a=this.h.XMLDocument.documentElement,b=a.attributes.length;0<b;b--)a.removeAttribute(a.attributes[b-1].nodeName);jg(this)};
function jg(a){try{a.h.save(a.i)}catch(b){throw"Storage mechanism: Quota exceeded";}}
;function kg(a,b){this.i=a;this.h=b+"::"}
Xa(kg,cg);kg.prototype.set=function(a,b){this.i.set(this.h+a,b)};
kg.prototype.get=function(a){return this.i.get(this.h+a)};
kg.prototype.remove=function(a){this.i.remove(this.h+a)};
kg.prototype.T=function(a){var b=this.i.T(!0),c=this,d=new gf;d.next=function(){try{var f=b.da()}catch(g){if(g===ff)return hf;throw g;}for(;f.substr(0,c.h.length)!=c.h;)try{f=b.da()}catch(g){if(g===ff)return hf;throw g;}return jf(a?f.substr(c.h.length):c.i.get(f))};
var e=d.next;d.da=function(){return kf(e.call(d))};
return d};function lg(a){H.call(this,a)}
r(lg,H);lg.prototype.getKey=function(){return Mc(this,1)};
lg.prototype.getValue=function(){return Mc(this,2===Pc(this,mg)?2:-1)};
lg.prototype.setValue=function(a){return Oc(this,2,mg,a)};
var mg=[2,3,4,5,6];function ng(a){H.call(this,a)}
r(ng,H);function og(a){H.call(this,a)}
r(og,H);function pg(a){H.call(this,a)}
r(pg,H);function qg(a){H.call(this,a,-1,rg)}
r(qg,H);qg.prototype.getPlayerType=function(){return Mc(this,36)};
qg.prototype.setHomeGroupInfo=function(a){return G(this,81,a)};
var rg=[9,66,24,32,86,100,101];function sg(a){H.call(this,a,-1,tg)}
r(sg,H);var tg=[15,26,28];function ug(a){H.call(this,a)}
r(ug,H);ug.prototype.setToken=function(a){return F(this,2,a)};function vg(a){H.call(this,a,-1,wg)}
r(vg,H);vg.prototype.setSafetyMode=function(a){return F(this,5,a)};
var wg=[12];function xg(a){H.call(this,a,-1,yg)}
r(xg,H);var yg=[12];function zg(a){H.call(this,a)}
r(zg,H);var Ag={vh:0,gh:1,mh:2,nh:4,sh:8,oh:16,ph:32,uh:64,th:128,ih:256,kh:512,rh:1024,jh:2048,lh:4096,hh:8192,qh:16384};function Bg(a){H.call(this,a)}
r(Bg,H);function Cg(a,b){G(a,1,b)}
Bg.prototype.Y=function(a){F(this,2,a)};
function Dg(a){H.call(this,a)}
r(Dg,H);function Eg(a,b){G(a,1,b)}
;function Fg(a){H.call(this,a,-1,Gg)}
r(Fg,H);Fg.prototype.Y=function(a){F(this,1,a)};
function Hg(a,b){G(a,2,b)}
var Gg=[3];function Ig(a){H.call(this,a)}
r(Ig,H);Ig.prototype.Y=function(a){F(this,1,a)};function Jg(a){H.call(this,a)}
r(Jg,H);Jg.prototype.Y=function(a){F(this,1,a)};function Kg(a){H.call(this,a)}
r(Kg,H);Kg.prototype.Y=function(a){F(this,1,a)};function Lg(a){H.call(this,a)}
r(Lg,H);function Mg(a){H.call(this,a)}
r(Mg,H);function Ng(a){H.call(this,a,-1,Og)}
r(Ng,H);Ng.prototype.getPlayerType=function(){var a=Mc(this,7);return null==a?0:a};
Ng.prototype.setVideoId=function(a){return F(this,19,a)};
function Pg(a,b){Uc(a,68,Qg,b)}
function Qg(a){H.call(this,a)}
r(Qg,H);Qg.prototype.getId=function(){var a=Mc(this,2);return null==a?"":a};
var Og=[83,68];function Rg(a){H.call(this,a)}
r(Rg,H);function Sg(a){H.call(this,a)}
r(Sg,H);function Tg(a){H.call(this,a)}
r(Tg,H);function Ug(a){H.call(this,a,427)}
r(Ug,H);
var Tc=[23,24,11,6,7,5,2,3,20,21,28,32,37,229,241,45,59,225,288,72,73,78,208,156,202,215,74,76,79,80,111,85,91,97,100,102,105,119,126,127,136,146,157,158,159,163,164,168,176,222,383,177,178,179,411,184,188,189,190,191,193,194,195,196,198,199,200,201,203,204,205,206,258,259,260,261,209,226,227,232,233,234,240,247,248,251,254,255,270,278,291,293,300,304,308,309,310,311,313,314,319,321,323,324,328,330,331,332,337,338,340,344,348,350,351,352,353,354,355,356,357,358,361,363,364,368,369,370,373,374,375,
378,380,381,388,389,403,412,413,414,415,416,417,418,419,420,423,424,425,426,117];function Vg(a){H.call(this,a)}
r(Vg,H);function Wg(a){H.call(this,a)}
r(Wg,H);Wg.prototype.setVideoId=function(a){return Oc(this,1,Xg,a)};
Wg.prototype.getPlaylistId=function(){return Mc(this,2===Pc(this,Xg)?2:-1)};
var Xg=[1,2];function Yg(a){H.call(this,a,-1,Zg)}
r(Yg,H);var Zg=[3];function $g(a){H.call(this,a,1)}
r($g,H);function ah(a){H.call(this,a)}
r(ah,H);var bh;bh=new function(a,b){this.h=a;this.fieldName=b;this.isRepeated=0;this.i=gd}(406606992,{yo:0},ah);function ch(){ah.apply(this,arguments)}
r(ch,ah);function dh(a,b){1<b.length?a[b[0]]=b[1]:1===b.length&&Object.assign(a,b[0])}
;var eh,fh,gh,hh=x.window,ih=(null===(eh=null===hh||void 0===hh?void 0:hh.yt)||void 0===eh?void 0:eh.config_)||(null===(fh=null===hh||void 0===hh?void 0:hh.ytcfg)||void 0===fh?void 0:fh.data_)||{},jh=(null===(gh=null===hh||void 0===hh?void 0:hh.ytcfg)||void 0===gh?void 0:gh.obfuscatedData_)||[];function kh(){$g.apply(this,arguments)}
r(kh,$g);var lh=new kh(jh),mh=ih.EXPERIMENT_FLAGS;if(!mh||!mh.jspb_i18n_extension){var nh=new ch;bh.i(lh,nh)}y("yt.config_",ih,void 0);y("yt.configJspb_",jh,void 0);function oh(){dh(ih,arguments)}
function B(a,b){return a in ih?ih[a]:b}
function ph(a){return B(a,void 0)}
;function K(a){a=qh(a);return"string"===typeof a&&"false"===a?!1:!!a}
function rh(a,b){a=qh(a);return void 0===a&&void 0!==b?b:Number(a||0)}
function sh(){return B("EXPERIMENTS_TOKEN","")}
function qh(a){var b=B("EXPERIMENTS_FORCED_FLAGS",{});return void 0!==b[a]?b[a]:B("EXPERIMENT_FLAGS",{})[a]}
function th(){var a=[],b=B("EXPERIMENTS_FORCED_FLAGS",{});for(c in b)a.push({key:c,value:String(b[c])});var c=B("EXPERIMENT_FLAGS",{});for(var d in c)d.startsWith("force_")&&void 0===b[d]&&a.push({key:d,value:String(c[d])});return a}
;var uh={appSettingsCaptured:!0,visualElementAttached:!0,visualElementGestured:!0,visualElementHidden:!0,visualElementShown:!0,flowEvent:!0,visualElementStateChanged:!0,playbackAssociated:!0,youThere:!0,accountStateChangeSignedIn:!0,accountStateChangeSignedOut:!0},vh={latencyActionBaselined:!0,latencyActionInfo:!0,latencyActionTicked:!0,bedrockRepetitiveActionTimed:!0,adsClientStateChange:!0,streamzIncremented:!0,mdxDialAdditionalDataUpdateEvent:!0,tvhtml5WatchKeyEvent:!0,tvhtml5VideoSeek:!0,tokenRefreshEvent:!0,
adNotify:!0,adNotifyFilled:!0,tvhtml5LaunchUrlComponentChanged:!0,bedrockResourceConsumptionSnapshot:!0,deviceStartupMetrics:!0,mdxSignIn:!0,tvhtml5KeyboardLogging:!0,tvhtml5StartupSoundEvent:!0,tvhtml5LiveChatStatus:!0,tvhtml5DeviceStorageStatus:!0,tvhtml5LocalStorage:!0,directSignInEvent:!0,finalPayload:!0,tvhtml5SearchCompleted:!0,tvhtml5KeyboardPerformance:!0,adNotifyFailure:!0,latencyActionSpan:!0,tvhtml5AccountDialogOpened:!0,tvhtml5ApiTest:!0};var wh=0,xh=jc?"webkit":ic?"moz":gc?"ms":fc?"o":"";y("ytDomDomGetNextId",A("ytDomDomGetNextId")||function(){return++wh},void 0);var yh=[];function zh(a){yh.forEach(function(b){return b(a)})}
function Ah(a){return a&&window.yterr?function(){try{return a.apply(this,arguments)}catch(b){M(b)}}:a}
function M(a,b,c,d){var e=A("yt.logging.errors.log");e?e(a,"ERROR",b,c,d):(e=B("ERRORS",[]),e.push([a,"ERROR",b,c,d]),oh("ERRORS",e));zh(a)}
function Bh(a,b,c,d){var e=A("yt.logging.errors.log");e?e(a,"WARNING",b,c,d):(e=B("ERRORS",[]),e.push([a,"WARNING",b,c,d]),oh("ERRORS",e))}
;var Ch={stopImmediatePropagation:1,stopPropagation:1,preventMouseEvent:1,preventManipulation:1,preventDefault:1,layerX:1,layerY:1,screenX:1,screenY:1,scale:1,rotation:1,webkitMovementX:1,webkitMovementY:1};
function Dh(a){this.type="";this.state=this.source=this.data=this.currentTarget=this.relatedTarget=this.target=null;this.charCode=this.keyCode=0;this.metaKey=this.shiftKey=this.ctrlKey=this.altKey=!1;this.rotation=this.clientY=this.clientX=0;this.scale=1;this.changedTouches=this.touches=null;try{if(a=a||window.event){this.event=a;for(var b in a)b in Ch||(this[b]=a[b]);this.scale=a.scale;this.rotation=a.rotation;var c=a.target||a.srcElement;c&&3==c.nodeType&&(c=c.parentNode);this.target=c;var d=a.relatedTarget;
if(d)try{d=d.nodeName?d:null}catch(e){d=null}else"mouseover"==this.type?d=a.fromElement:"mouseout"==this.type&&(d=a.toElement);this.relatedTarget=d;this.clientX=void 0!=a.clientX?a.clientX:a.pageX;this.clientY=void 0!=a.clientY?a.clientY:a.pageY;this.keyCode=a.keyCode?a.keyCode:a.which;this.charCode=a.charCode||("keypress"==this.type?this.keyCode:0);this.altKey=a.altKey;this.ctrlKey=a.ctrlKey;this.shiftKey=a.shiftKey;this.metaKey=a.metaKey;this.h=a.pageX;this.i=a.pageY}}catch(e){}}
function Eh(a){if(document.body&&document.documentElement){var b=document.body.scrollTop+document.documentElement.scrollTop;a.h=a.clientX+(document.body.scrollLeft+document.documentElement.scrollLeft);a.i=a.clientY+b}}
Dh.prototype.preventDefault=function(){this.event&&(this.event.returnValue=!1,this.event.preventDefault&&this.event.preventDefault())};
Dh.prototype.stopPropagation=function(){this.event&&(this.event.cancelBubble=!0,this.event.stopPropagation&&this.event.stopPropagation())};
Dh.prototype.stopImmediatePropagation=function(){this.event&&(this.event.cancelBubble=!0,this.event.stopImmediatePropagation&&this.event.stopImmediatePropagation())};var nb=x.ytEventsEventsListeners||{};y("ytEventsEventsListeners",nb,void 0);var Fh=x.ytEventsEventsCounter||{count:0};y("ytEventsEventsCounter",Fh,void 0);
function Gh(a,b,c,d){d=void 0===d?{}:d;a.addEventListener&&("mouseenter"!=b||"onmouseenter"in document?"mouseleave"!=b||"onmouseenter"in document?"mousewheel"==b&&"MozBoxSizing"in document.documentElement.style&&(b="MozMousePixelScroll"):b="mouseout":b="mouseover");return mb(function(e){var f="boolean"===typeof e[4]&&e[4]==!!d,g=Oa(e[4])&&Oa(d)&&rb(e[4],d);return!!e.length&&e[0]==a&&e[1]==b&&e[2]==c&&(f||g)})}
var Hh=bb(function(){var a=!1;try{var b=Object.defineProperty({},"capture",{get:function(){a=!0}});
window.addEventListener("test",null,b)}catch(c){}return a});
function Ih(a,b,c,d){d=void 0===d?{}:d;if(!a||!a.addEventListener&&!a.attachEvent)return"";var e=Gh(a,b,c,d);if(e)return e;e=++Fh.count+"";var f=!("mouseenter"!=b&&"mouseleave"!=b||!a.addEventListener||"onmouseenter"in document);var g=f?function(h){h=new Dh(h);if(!wd(h.relatedTarget,function(k){return k==a}))return h.currentTarget=a,h.type=b,c.call(a,h)}:function(h){h=new Dh(h);
h.currentTarget=a;return c.call(a,h)};
g=Ah(g);a.addEventListener?("mouseenter"==b&&f?b="mouseover":"mouseleave"==b&&f?b="mouseout":"mousewheel"==b&&"MozBoxSizing"in document.documentElement.style&&(b="MozMousePixelScroll"),Hh()||"boolean"===typeof d?a.addEventListener(b,g,d):a.addEventListener(b,g,!!d.capture)):a.attachEvent("on"+b,g);nb[e]=[a,b,c,g,d];return e}
function Jh(a){a&&("string"==typeof a&&(a=[a]),db(a,function(b){if(b in nb){var c=nb[b],d=c[0],e=c[1],f=c[3];c=c[4];d.removeEventListener?Hh()||"boolean"===typeof c?d.removeEventListener(e,f,c):d.removeEventListener(e,f,!!c.capture):d.detachEvent&&d.detachEvent("on"+e,f);delete nb[b]}}))}
;var Kh=window.ytcsi&&window.ytcsi.now?window.ytcsi.now:window.performance&&window.performance.timing&&window.performance.now&&window.performance.timing.navigationStart?function(){return window.performance.timing.navigationStart+window.performance.now()}:function(){return(new Date).getTime()};function Lh(a,b){"function"===typeof a&&(a=Ah(a));return window.setTimeout(a,b)}
function Mh(a){window.clearTimeout(a)}
;function Nh(a){this.L=a;this.i=null;this.m=0;this.A=null;this.u=0;this.j=[];for(a=0;4>a;a++)this.j.push(0);this.l=0;this.S=Ih(window,"mousemove",Ua(this.Z,this));a=Ua(this.M,this);"function"===typeof a&&(a=Ah(a));this.ga=window.setInterval(a,25)}
Xa(Nh,I);Nh.prototype.Z=function(a){void 0===a.h&&Eh(a);var b=a.h;void 0===a.i&&Eh(a);this.i=new sd(b,a.i)};
Nh.prototype.M=function(){if(this.i){var a=Kh();if(0!=this.m){var b=this.A,c=this.i,d=b.x-c.x;b=b.y-c.y;d=Math.sqrt(d*d+b*b)/(a-this.m);this.j[this.l]=.5<Math.abs((d-this.u)/this.u)?1:0;for(c=b=0;4>c;c++)b+=this.j[c]||0;3<=b&&this.L();this.u=d}this.m=a;this.A=this.i;this.l=(this.l+1)%4}};
Nh.prototype.I=function(){window.clearInterval(this.ga);Jh(this.S)};function Oh(){}
function Ph(a,b){return Qh(a,0,b)}
Oh.prototype.N=function(a,b){return Qh(a,1,b)};
function Rh(a,b){Qh(a,2,b)}
function Sh(a){var b=A("yt.scheduler.instance.addImmediateJob");b?b(a):a()}
;function Th(){Oh.apply(this,arguments)}
r(Th,Oh);function Uh(){Th.h||(Th.h=new Th);return Th.h}
function Qh(a,b,c){void 0!==c&&Number.isNaN(Number(c))&&(c=void 0);var d=A("yt.scheduler.instance.addJob");return d?d(a,b,c):void 0===c?(a(),NaN):Lh(a,c||0)}
Th.prototype.U=function(a){if(void 0===a||!Number.isNaN(Number(a))){var b=A("yt.scheduler.instance.cancelJob");b?b(a):Mh(a)}};
Th.prototype.start=function(){var a=A("yt.scheduler.instance.start");a&&a()};
Th.prototype.pause=function(){var a=A("yt.scheduler.instance.pause");a&&a()};var Vh=Uh();var Wh={};
function Xh(a){var b=void 0===a?{}:a;a=void 0===b.ec?!1:b.ec;b=void 0===b.Tb?!0:b.Tb;if(null==A("_lact",window)){var c=parseInt(B("LACT"),10);c=isFinite(c)?Date.now()-Math.max(c,0):-1;y("_lact",c,window);y("_fact",c,window);-1==c&&Yh();Ih(document,"keydown",Yh);Ih(document,"keyup",Yh);Ih(document,"mousedown",Yh);Ih(document,"mouseup",Yh);a?Ih(window,"touchmove",function(){Zh("touchmove",200)},{passive:!0}):(Ih(window,"resize",function(){Zh("resize",200)}),b&&Ih(window,"scroll",function(){Zh("scroll",200)}));
new Nh(function(){Zh("mouse",100)});
Ih(document,"touchstart",Yh,{passive:!0});Ih(document,"touchend",Yh,{passive:!0})}}
function Zh(a,b){Wh[a]||(Wh[a]=!0,Vh.N(function(){Yh();Wh[a]=!1},b))}
function Yh(){null==A("_lact",window)&&Xh();var a=Date.now();y("_lact",a,window);-1==A("_fact",window)&&y("_fact",a,window);(a=A("ytglobal.ytUtilActivityCallback_"))&&a()}
function $h(){var a=A("_lact",window);return null==a?-1:Math.max(Date.now()-a,0)}
;function ai(){var a=bi;A("yt.ads.biscotti.getId_")||y("yt.ads.biscotti.getId_",a,void 0)}
function ci(a){y("yt.ads.biscotti.lastId_",a,void 0)}
;var di=/^[\w.]*$/,ei={q:!0,search_query:!0};function fi(a,b){b=a.split(b);for(var c={},d=0,e=b.length;d<e;d++){var f=b[d].split("=");if(1==f.length&&f[0]||2==f.length)try{var g=gi(f[0]||""),h=gi(f[1]||"");g in c?Array.isArray(c[g])?kb(c[g],h):c[g]=[c[g],h]:c[g]=h}catch(t){var k=t,l=f[0],n=String(fi);k.args=[{key:l,value:f[1],query:a,method:hi==n?"unchanged":n}];ei.hasOwnProperty(l)||Bh(k)}}return c}
var hi=String(fi);function ii(a){var b=[];lb(a,function(c,d){var e=encodeURIComponent(String(d)),f;Array.isArray(c)?f=c:f=[c];db(f,function(g){""==g?b.push(e):b.push(e+"="+encodeURIComponent(String(g)))})});
return b.join("&")}
function ji(a){"?"==a.charAt(0)&&(a=a.substr(1));return fi(a,"&")}
function ki(a){return-1!=a.indexOf("?")?(a=(a||"").split("#")[0],a=a.split("?",2),ji(1<a.length?a[1]:a[0])):{}}
function li(a,b,c){var d=a.split("#",2);a=d[0];d=1<d.length?"#"+d[1]:"";var e=a.split("?",2);a=e[0];e=ji(e[1]||"");for(var f in b)!c&&null!==e&&f in e||(e[f]=b[f]);return bc(a,e)+d}
function mi(a){if(!b)var b=window.location.href;var c=a.match(Wb)[1]||null,d=Yb(a);c&&d?(a=a.match(Wb),b=b.match(Wb),a=a[3]==b[3]&&a[1]==b[1]&&a[4]==b[4]):a=d?Yb(b)==d&&(Number(b.match(Wb)[4]||null)||null)==(Number(a.match(Wb)[4]||null)||null):!0;return a}
function gi(a){return a&&a.match(di)?a:decodeURIComponent(a.replace(/\+/g," "))}
;function ni(a){var b=oi;a=void 0===a?A("yt.ads.biscotti.lastId_")||"":a;var c=Object,d=c.assign,e={};e.dt=Id;e.flash="0";a:{try{var f=b.h.top.location.href}catch(jb){f=2;break a}f=f?f===b.i.location.href?0:1:2}e=(e.frm=f,e);try{e.u_tz=-(new Date).getTimezoneOffset();var g=void 0===g?ld:g;try{var h=g.history.length}catch(jb){h=0}e.u_his=h;var k;e.u_h=null==(k=ld.screen)?void 0:k.height;var l;e.u_w=null==(l=ld.screen)?void 0:l.width;var n;e.u_ah=null==(n=ld.screen)?void 0:n.availHeight;var t;e.u_aw=
null==(t=ld.screen)?void 0:t.availWidth;var z;e.u_cd=null==(z=ld.screen)?void 0:z.colorDepth}catch(jb){}h=b.h;try{var u=h.screenX;var D=h.screenY}catch(jb){}try{var E=h.outerWidth;var L=h.outerHeight}catch(jb){}try{var P=h.innerWidth;var S=h.innerHeight}catch(jb){}try{var W=h.screenLeft;var Z=h.screenTop}catch(jb){}try{P=h.innerWidth,S=h.innerHeight}catch(jb){}try{var Cd=h.screen.availWidth;var ea=h.screen.availTop}catch(jb){}u=[W,Z,u,D,Cd,ea,E,L,P,S];try{var ia=(b.h.top||window).document,ua="CSS1Compat"==
ia.compatMode?ia.documentElement:ia.body;var Za=(new td(ua.clientWidth,ua.clientHeight)).round()}catch(jb){Za=new td(-12245933,-12245933)}ia=Za;Za={};ua=new Ie;x.SVGElement&&x.document.createElementNS&&ua.set(0);D=Bd();D["allow-top-navigation-by-user-activation"]&&ua.set(1);D["allow-popups-to-escape-sandbox"]&&ua.set(2);x.crypto&&x.crypto.subtle&&ua.set(3);x.TextDecoder&&x.TextEncoder&&ua.set(4);ua=Je(ua);Za.bc=ua;Za.bih=ia.height;Za.biw=ia.width;Za.brdim=u.join();b=b.i;b=(Za.vis=b.prerendering?3:
{visible:1,hidden:2,prerender:3,preview:4,unloaded:5}[b.visibilityState||b.webkitVisibilityState||b.mozVisibilityState||""]||0,Za.wgl=!!ld.WebGLRenderingContext,Za);c=d.call(c,e,b);c.ca_type="image";a&&(c.bid=a);return c}
var oi=new function(){var a=window.document;this.h=window;this.i=a};
y("yt.ads_.signals_.getAdSignalsString",function(a){return ii(ni(a))},void 0);Date.now();var pi="XMLHttpRequest"in x?function(){return new XMLHttpRequest}:null;
function qi(){if(!pi)return null;var a=pi();return"open"in a?a:null}
function ri(a){switch(a&&"status"in a?a.status:-1){case 200:case 201:case 202:case 203:case 204:case 205:case 206:case 304:return!0;default:return!1}}
;var si={Authorization:"AUTHORIZATION","X-Goog-Visitor-Id":"SANDBOXED_VISITOR_ID","X-Youtube-Domain-Admin-State":"DOMAIN_ADMIN_STATE","X-Youtube-Chrome-Connected":"CHROME_CONNECTED_HEADER","X-YouTube-Client-Name":"INNERTUBE_CONTEXT_CLIENT_NAME","X-YouTube-Client-Version":"INNERTUBE_CONTEXT_CLIENT_VERSION","X-YouTube-Delegation-Context":"INNERTUBE_CONTEXT_SERIALIZED_DELEGATION_CONTEXT","X-YouTube-Device":"DEVICE","X-Youtube-Identity-Token":"ID_TOKEN","X-YouTube-Page-CL":"PAGE_CL","X-YouTube-Page-Label":"PAGE_BUILD_LABEL",
"X-YouTube-Variants-Checksum":"VARIANTS_CHECKSUM"},ti="app debugcss debugjs expflag force_ad_params force_ad_encrypted force_viral_ad_response_params forced_experiments innertube_snapshots innertube_goldens internalcountrycode internalipoverride absolute_experiments conditional_experiments sbb sr_bns_address".split(" ").concat(ha("client_dev_mss_url client_dev_regex_map client_dev_root_url client_rollout_override expflag jsfeat jsmode mods".split(" "))),ui=!1;
function vi(a,b){b=void 0===b?{}:b;var c=mi(a),d=K("web_ajax_ignore_global_headers_if_set"),e;for(e in si){var f=B(si[e]);!f||!c&&Yb(a)||d&&void 0!==b[e]||(b[e]=f)}if(c||!Yb(a))b["X-YouTube-Utc-Offset"]=String(-(new Date).getTimezoneOffset());if(c||!Yb(a)){try{var g=(new Intl.DateTimeFormat).resolvedOptions().timeZone}catch(h){}g&&(b["X-YouTube-Time-Zone"]=g)}if(c||!Yb(a))b["X-YouTube-Ad-Signals"]=ii(ni(void 0));return b}
function wi(a){var b=window.location.search,c=Yb(a);K("debug_handle_relative_url_for_query_forward_killswitch")||c||!mi(a)||(c=document.location.hostname);var d=Xb(a.match(Wb)[5]||null);d=(c=c&&(c.endsWith("youtube.com")||c.endsWith("youtube-nocookie.com")))&&d&&d.startsWith("/api/");if(!c||d)return a;var e=ji(b),f={};db(ti,function(g){e[g]&&(f[g]=e[g])});
return li(a,f||{},!1)}
function xi(a,b){var c=b.format||"JSON";a=yi(a,b);var d=zi(a,b),e=!1,f=Ai(a,function(k){if(!e){e=!0;h&&Mh(h);var l=ri(k),n=null,t=400<=k.status&&500>k.status,z=500<=k.status&&600>k.status;if(l||t||z)n=Bi(a,c,k,b.convertToSafeHtml);if(l)a:if(k&&204==k.status)l=!0;else{switch(c){case "XML":l=0==parseInt(n&&n.return_code,10);break a;case "RAW":l=!0;break a}l=!!n}n=n||{};t=b.context||x;l?b.onSuccess&&b.onSuccess.call(t,k,n):b.onError&&b.onError.call(t,k,n);b.onFinish&&b.onFinish.call(t,k,n)}},b.method,
d,b.headers,b.responseType,b.withCredentials);
if(b.onTimeout&&0<b.timeout){var g=b.onTimeout;var h=Lh(function(){e||(e=!0,f.abort(),Mh(h),g.call(b.context||x,f))},b.timeout)}return f}
function yi(a,b){b.includeDomain&&(a=document.location.protocol+"//"+document.location.hostname+(document.location.port?":"+document.location.port:"")+a);var c=B("XSRF_FIELD_NAME",void 0);if(b=b.urlParams)b[c]&&delete b[c],a=li(a,b||{},!0);return a}
function zi(a,b){var c=B("XSRF_FIELD_NAME",void 0),d=B("XSRF_TOKEN",void 0),e=b.postBody||"",f=b.postParams,g=ph("XSRF_FIELD_NAME"),h;b.headers&&(h=b.headers["Content-Type"]);b.excludeXsrf||Yb(a)&&!b.withCredentials&&Yb(a)!=document.location.hostname||"POST"!=b.method||h&&"application/x-www-form-urlencoded"!=h||b.postParams&&b.postParams[g]||(f||(f={}),f[c]=d);f&&"string"===typeof e&&(e=ji(e),vb(e,f),e=b.postBodyFormat&&"JSON"==b.postBodyFormat?JSON.stringify(e):ac(e));f=e||f&&!ob(f);!ui&&f&&"POST"!=
b.method&&(ui=!0,M(Error("AJAX request with postData should use POST")));return e}
function Bi(a,b,c,d){var e=null;switch(b){case "JSON":try{var f=c.responseText}catch(g){throw d=Error("Error reading responseText"),d.params=a,Bh(d),g;}a=c.getResponseHeader("Content-Type")||"";f&&0<=a.indexOf("json")&&(")]}'\n"===f.substring(0,5)&&(f=f.substring(5)),e=JSON.parse(f));break;case "XML":if(a=(a=c.responseXML)?Ci(a):null)e={},db(a.getElementsByTagName("*"),function(g){e[g.tagName]=Di(g)})}d&&Ei(e);
return e}
function Ei(a){if(Oa(a))for(var b in a){var c;(c="html_content"==b)||(c=b.length-5,c=0<=c&&b.indexOf("_html",c)==c);if(c){c=b;Ab("HTML that is escaped and sanitized server-side and passed through yt.net.ajax");var d=a[b],e=xb();d=e?e.createHTML(d):d;a[c]=new Pb(d)}else Ei(a[b])}}
function Ci(a){return a?(a=("responseXML"in a?a.responseXML:a).getElementsByTagName("root"))&&0<a.length?a[0]:null:null}
function Di(a){var b="";db(a.childNodes,function(c){b+=c.nodeValue});
return b}
function Ai(a,b,c,d,e,f,g){function h(){4==(k&&"readyState"in k?k.readyState:0)&&b&&Ah(b)(k)}
c=void 0===c?"GET":c;d=void 0===d?"":d;var k=qi();if(!k)return null;"onloadend"in k?k.addEventListener("loadend",h,!1):k.onreadystatechange=h;K("debug_forward_web_query_parameters")&&(a=wi(a));k.open(c,a,!0);f&&(k.responseType=f);g&&(k.withCredentials=!0);c="POST"==c&&(void 0===window.FormData||!(d instanceof FormData));if(e=vi(a,e))for(var l in e)k.setRequestHeader(l,e[l]),"content-type"==l.toLowerCase()&&(c=!1);c&&k.setRequestHeader("Content-Type","application/x-www-form-urlencoded");k.send(d);
return k}
;var Fi=x.ytPubsubPubsubInstance||new J,Gi=x.ytPubsubPubsubSubscribedKeys||{},Hi=x.ytPubsubPubsubTopicToKeys||{},Ii=x.ytPubsubPubsubIsSynchronous||{};function Ji(a,b){var c=Ki();if(c&&b){var d=c.subscribe(a,function(){var e=arguments;var f=function(){Gi[d]&&b.apply&&"function"==typeof b.apply&&b.apply(window,e)};
try{Ii[a]?f():Lh(f,0)}catch(g){M(g)}},void 0);
Gi[d]=!0;Hi[a]||(Hi[a]=[]);Hi[a].push(d);return d}return 0}
function Li(a){var b=Ki();b&&("number"===typeof a?a=[a]:"string"===typeof a&&(a=[parseInt(a,10)]),db(a,function(c){b.unsubscribeByKey(c);delete Gi[c]}))}
function Mi(a,b){var c=Ki();c&&c.publish.apply(c,arguments)}
function Ni(a){var b=Ki();if(b)if(b.clear(a),a)Oi(a);else for(var c in Hi)Oi(c)}
function Ki(){return x.ytPubsubPubsubInstance}
function Oi(a){Hi[a]&&(a=Hi[a],db(a,function(b){Gi[b]&&delete Gi[b]}),a.length=0)}
J.prototype.subscribe=J.prototype.subscribe;J.prototype.unsubscribeByKey=J.prototype.va;J.prototype.publish=J.prototype.ka;J.prototype.clear=J.prototype.clear;y("ytPubsubPubsubInstance",Fi,void 0);y("ytPubsubPubsubTopicToKeys",Hi,void 0);y("ytPubsubPubsubIsSynchronous",Ii,void 0);y("ytPubsubPubsubSubscribedKeys",Gi,void 0);var Pi=window,N=Pi.ytcsi&&Pi.ytcsi.now?Pi.ytcsi.now:Pi.performance&&Pi.performance.timing&&Pi.performance.now&&Pi.performance.timing.navigationStart?function(){return Pi.performance.timing.navigationStart+Pi.performance.now()}:function(){return(new Date).getTime()};var Qi=rh("initial_gel_batch_timeout",2E3),Ri=Math.pow(2,16)-1,Si=void 0;function Ti(){this.j=this.h=this.i=0}
var Ui=new Ti,Vi=new Ti,Wi=!0,Xi=x.ytLoggingTransportGELQueue_||new Map;y("ytLoggingTransportGELQueue_",Xi,void 0);var Yi=x.ytLoggingTransportGELProtoQueue_||new Map;y("ytLoggingTransportGELProtoQueue_",Yi,void 0);var Zi=x.ytLoggingTransportTokensToCttTargetIds_||{};y("ytLoggingTransportTokensToCttTargetIds_",Zi,void 0);var $i=x.ytLoggingTransportTokensToJspbCttTargetIds_||{};y("ytLoggingTransportTokensToJspbCttTargetIds_",$i,void 0);
function aj(a,b){if("log_event"===a.endpoint){var c=bj(a),d=Xi.get(c)||[];Xi.set(c,d);d.push(a.payload);cj(b,d,c)}}
function dj(a,b){if("log_event"===a.endpoint){var c=bj(a,!0),d=Yi.get(c)||[];Yi.set(c,d);d.push(a.payload);cj(b,d,c,!0)}}
function cj(a,b,c,d){d=void 0===d?!1:d;a&&(Si=new a);a=rh("tvhtml5_logging_max_batch")||rh("web_logging_max_batch")||100;var e=N(),f=d?Vi.j:Ui.j;b.length>=a?ej({writeThenSend:!0},K("flush_only_full_queue")?c:void 0,d):10<=e-f&&(fj(d),d?Vi.j=e:Ui.j=e)}
function gj(a,b){if("log_event"===a.endpoint){var c=bj(a),d=new Map;d.set(c,[a.payload]);b&&(Si=new b);return new Df(function(e){Si&&Si.isReady()?hj(d,e,{bypassNetworkless:!0},!0):e()})}}
function ij(a,b){if("log_event"===a.endpoint){var c=bj(a,!0),d=new Map;d.set(c,[a.payload]);b&&(Si=new b);return new Df(function(e){Si&&Si.isReady()?jj(d,e,{bypassNetworkless:!0},!0):e()})}}
function bj(a,b){var c="";if(a.wa)c="visitorOnlyApprovedKey";else if(a.cttAuthInfo){if(void 0===b?0:b){b=a.cttAuthInfo.token;c=a.cttAuthInfo;var d=new Wg;c.videoId?d.setVideoId(c.videoId):c.playlistId&&Oc(d,2,Xg,c.playlistId);$i[b]=d}else b=a.cttAuthInfo,c={},b.videoId?c.videoId=b.videoId:b.playlistId&&(c.playlistId=b.playlistId),Zi[a.cttAuthInfo.token]=c;c=a.cttAuthInfo.token}return c}
function ej(a,b,c){a=void 0===a?{}:a;c=void 0===c?!1:c;new Df(function(d){c?(Mh(Vi.i),Mh(Vi.h),Vi.h=0):(Mh(Ui.i),Mh(Ui.h),Ui.h=0);if(Si&&Si.isReady())if(void 0!==b)if(c){var e=new Map,f=Yi.get(b)||[];e.set(b,f);jj(e,d,a);Yi.delete(b)}else e=new Map,f=Xi.get(b)||[],e.set(b,f),hj(e,d,a),Xi.delete(b);else c?(jj(Yi,d,a),Yi.clear()):(hj(Xi,d,a),Xi.clear());else fj(c),d()})}
function fj(a){a=void 0===a?!1:a;if(K("web_gel_timeout_cap")&&(!a&&!Ui.h||a&&!Vi.h)){var b=Lh(function(){ej({writeThenSend:!0},void 0,a)},6E4);
a?Vi.h=b:Ui.h=b}Mh(a?Vi.i:Ui.i);b=B("LOGGING_BATCH_TIMEOUT",rh("web_gel_debounce_ms",1E4));K("shorten_initial_gel_batch_timeout")&&Wi&&(b=Qi);b=Lh(function(){ej({writeThenSend:!0},void 0,a)},b);
a?Vi.i=b:Ui.i=b}
function hj(a,b,c,d){var e=Si;c=void 0===c?{}:c;var f=Math.round(N()),g=a.size;a=q(a);for(var h=a.next();!h.done;h=a.next()){var k=q(h.value);h=k.next().value;var l=k=k.next().value;k=tb({context:kj(e.config_||lj())});k.events=l;(l=Zi[h])&&mj(k,h,l);delete Zi[h];h="visitorOnlyApprovedKey"===h;nj(k,f,h);oj(c);pj(e,"log_event",k,qj(c,h,function(){g--;g||b()},function(){g--;
g||b()},d));
Wi=!1}}
function jj(a,b,c,d){var e=Si;c=void 0===c?{}:c;var f=Math.round(N()),g=a.size;a=q(a);for(var h=a.next();!h.done;h=a.next()){var k=q(h.value);h=k.next().value;var l=k=k.next().value;k=new Yg;var n=rj(e.config_||lj());G(k,1,n);for(n=0;n<l.length;n++)Uc(k,3,Ug,l[n]);(l=$i[h])&&sj(k,h,l);delete $i[h];h="visitorOnlyApprovedKey"===h;tj(k,f,h);oj(c);a:{Ic=!0;try{var t=JSON.stringify(k.toJSON(),dd);break a}finally{Ic=!1}t=void 0}k=t;h=qj(c,h,function(){g--;g||b()},function(){g--;
g||b()},d);
h.headers={"Content-Type":"application/json+protobuf"};h.postBodyFormat="JSPB";h.postBody=k;pj(e,"log_event","",h);Wi=!1}}
function oj(a){K("always_send_and_write")&&(a.writeThenSend=!1)}
function qj(a,b,c,d,e){return{retry:!0,onSuccess:c,onError:d,Bb:a,wa:b,vo:!!e,headers:{},postBodyFormat:"",postBody:""}}
function nj(a,b,c){a.requestTimeMs=String(b);K("unsplit_gel_payloads_in_logs")&&(a.unsplitGelPayloadsInLogs=!0);!c&&(b=B("EVENT_ID",void 0))&&(c=uj(),a.serializedClientEventId={serializedEventId:b,clientCounter:String(c)})}
function tj(a,b,c){F(a,2,b);if(!c&&(b=B("EVENT_ID",void 0))){c=uj();var d=new Vg;F(d,1,b);F(d,2,c);G(a,5,d)}}
function uj(){var a=B("BATCH_CLIENT_COUNTER",void 0)||0;a||(a=Math.floor(Math.random()*Ri/2));a++;a>Ri&&(a=1);oh("BATCH_CLIENT_COUNTER",a);return a}
function mj(a,b,c){if(c.videoId)var d="VIDEO";else if(c.playlistId)d="PLAYLIST";else return;a.credentialTransferTokenTargetId=c;a.context=a.context||{};a.context.user=a.context.user||{};a.context.user.credentialTransferTokens=[{token:b,scope:d}]}
function sj(a,b,c){if(Mc(c,1===Pc(c,Xg)?1:-1))var d=1;else if(c.getPlaylistId())d=2;else return;G(a,4,c);a=Qc(a,xg,1)||new xg;c=Qc(a,vg,3)||new vg;var e=new ug;e.setToken(b);F(e,1,d);Uc(c,12,ug,e);G(a,3,c)}
;var vj=x.ytLoggingGelSequenceIdObj_||{};y("ytLoggingGelSequenceIdObj_",vj,void 0);
function wj(a,b,c,d){d=void 0===d?{}:d;if(K("lr_drop_other_and_business_payloads")){if(vh[a]||uh[a])return}else if(K("lr_drop_other_payloads")&&vh[a])return;var e={},f=Math.round(d.timestamp||N());e.eventTimeMs=f<Number.MAX_SAFE_INTEGER?f:0;e[a]=b;a=$h();e.context={lastActivityMs:String(d.timestamp||!isFinite(a)?-1:a)};K("log_sequence_info_on_gel_web")&&d.X&&(a=e.context,b=d.X,b={index:xj(b),groupKey:b},a.sequence=b,d.ub&&delete vj[d.X]);(d.mc?gj:aj)({endpoint:"log_event",payload:e,cttAuthInfo:d.cttAuthInfo,
wa:d.wa},c)}
function xj(a){vj[a]=a in vj?vj[a]+1:0;return vj[a]}
;function yj(a){var b=this;this.h=void 0;a.addEventListener("beforeinstallprompt",function(c){c.preventDefault();b.h=c})}
function zj(){if(!x.matchMedia)return"WEB_DISPLAY_MODE_UNKNOWN";try{return x.matchMedia("(display-mode: standalone)").matches?"WEB_DISPLAY_MODE_STANDALONE":x.matchMedia("(display-mode: minimal-ui)").matches?"WEB_DISPLAY_MODE_MINIMAL_UI":x.matchMedia("(display-mode: fullscreen)").matches?"WEB_DISPLAY_MODE_FULLSCREEN":x.matchMedia("(display-mode: browser)").matches?"WEB_DISPLAY_MODE_BROWSER":"WEB_DISPLAY_MODE_UNKNOWN"}catch(a){return"WEB_DISPLAY_MODE_UNKNOWN"}}
function Aj(){var a=zj();a=Object.keys(He).indexOf(a);return-1===a?null:a}
;function Bj(a,b,c,d,e){Rd.set(""+a,b,{Pa:c,path:"/",domain:void 0===d?"youtube.com":d,secure:void 0===e?!1:e})}
function Cj(a){return Rd.get(""+a,void 0)}
function Dj(){if(!Rd.isEnabled())return!1;if(!Rd.isEmpty())return!0;Rd.set("TESTCOOKIESENABLED","1",{Pa:60});if("1"!==Rd.get("TESTCOOKIESENABLED"))return!1;Rd.remove("TESTCOOKIESENABLED");return!0}
;var Ej=A("ytglobal.prefsUserPrefsPrefs_")||{};y("ytglobal.prefsUserPrefsPrefs_",Ej,void 0);function Fj(){this.h=B("ALT_PREF_COOKIE_NAME","PREF");this.i=B("ALT_PREF_COOKIE_DOMAIN","youtube.com");var a=Cj(this.h);if(a){a=decodeURIComponent(a).split("&");for(var b=0;b<a.length;b++){var c=a[b].split("="),d=c[0];(c=c[1])&&(Ej[d]=c.toString())}}}
Fj.prototype.get=function(a,b){Gj(a);Hj(a);a=void 0!==Ej[a]?Ej[a].toString():null;return null!=a?a:b?b:""};
Fj.prototype.set=function(a,b){Gj(a);Hj(a);if(null==b)throw Error("ExpectedNotNull");Ej[a]=b.toString()};
function Ij(a){return!!((Jj("f"+(Math.floor(a/31)+1))||0)&1<<a%31)}
Fj.prototype.remove=function(a){Gj(a);Hj(a);delete Ej[a]};
Fj.prototype.clear=function(){for(var a in Ej)delete Ej[a]};
function Hj(a){if(/^f([1-9][0-9]*)$/.test(a))throw Error("ExpectedRegexMatch: "+a);}
function Gj(a){if(!/^\w+$/.test(a))throw Error("ExpectedRegexMismatch: "+a);}
function Jj(a){a=void 0!==Ej[a]?Ej[a].toString():null;return null!=a&&/^[A-Fa-f0-9]+$/.test(a)?parseInt(a,16):null}
La(Fj);var Kj={bluetooth:"CONN_DISCO",cellular:"CONN_CELLULAR_UNKNOWN",ethernet:"CONN_WIFI",none:"CONN_NONE",wifi:"CONN_WIFI",wimax:"CONN_CELLULAR_4G",other:"CONN_UNKNOWN",unknown:"CONN_UNKNOWN","slow-2g":"CONN_CELLULAR_2G","2g":"CONN_CELLULAR_2G","3g":"CONN_CELLULAR_3G","4g":"CONN_CELLULAR_4G"},Lj={CONN_DEFAULT:0,CONN_UNKNOWN:1,CONN_NONE:2,CONN_WIFI:3,CONN_CELLULAR_2G:4,CONN_CELLULAR_3G:5,CONN_CELLULAR_4G:6,CONN_CELLULAR_UNKNOWN:7,CONN_DISCO:8,CONN_CELLULAR_5G:9,CONN_WIFI_METERED:10,CONN_CELLULAR_5G_SA:11,
CONN_CELLULAR_5G_NSA:12,CONN_INVALID:31},Mj={EFFECTIVE_CONNECTION_TYPE_UNKNOWN:0,EFFECTIVE_CONNECTION_TYPE_OFFLINE:1,EFFECTIVE_CONNECTION_TYPE_SLOW_2G:2,EFFECTIVE_CONNECTION_TYPE_2G:3,EFFECTIVE_CONNECTION_TYPE_3G:4,EFFECTIVE_CONNECTION_TYPE_4G:5},Nj={"slow-2g":"EFFECTIVE_CONNECTION_TYPE_SLOW_2G","2g":"EFFECTIVE_CONNECTION_TYPE_2G","3g":"EFFECTIVE_CONNECTION_TYPE_3G","4g":"EFFECTIVE_CONNECTION_TYPE_4G"};function Oj(){var a=x.navigator;return a?a.connection:void 0}
function Pj(){var a=Oj();if(a){var b=Kj[a.type||"unknown"]||"CONN_UNKNOWN";a=Kj[a.effectiveType||"unknown"]||"CONN_UNKNOWN";"CONN_CELLULAR_UNKNOWN"===b&&"CONN_UNKNOWN"!==a&&(b=a);if("CONN_UNKNOWN"!==b)return b;if("CONN_UNKNOWN"!==a)return a}}
function Qj(){var a=Oj();if(null!==a&&void 0!==a&&a.effectiveType)return Nj.hasOwnProperty(a.effectiveType)?Nj[a.effectiveType]:"EFFECTIVE_CONNECTION_TYPE_UNKNOWN"}
;function Rj(){return"INNERTUBE_API_KEY"in ih&&"INNERTUBE_API_VERSION"in ih}
function lj(){return{innertubeApiKey:B("INNERTUBE_API_KEY",void 0),innertubeApiVersion:B("INNERTUBE_API_VERSION",void 0),bb:B("INNERTUBE_CONTEXT_CLIENT_CONFIG_INFO"),cb:B("INNERTUBE_CONTEXT_CLIENT_NAME","WEB"),Vb:B("INNERTUBE_CONTEXT_CLIENT_NAME",1),innertubeContextClientVersion:B("INNERTUBE_CONTEXT_CLIENT_VERSION",void 0),yb:B("INNERTUBE_CONTEXT_HL",void 0),xb:B("INNERTUBE_CONTEXT_GL",void 0),Wb:B("INNERTUBE_HOST_OVERRIDE",void 0)||"",Yb:!!B("INNERTUBE_USE_THIRD_PARTY_AUTH",!1),Xb:!!B("INNERTUBE_OMIT_API_KEY_WHEN_AUTH_HEADER_IS_PRESENT",
!1),appInstallData:B("SERIALIZED_CLIENT_CONFIG_DATA",void 0)}}
function kj(a){var b={client:{hl:a.yb,gl:a.xb,clientName:a.cb,clientVersion:a.innertubeContextClientVersion,configInfo:a.bb}};navigator.userAgent&&(b.client.userAgent=String(navigator.userAgent));var c=x.devicePixelRatio;c&&1!=c&&(b.client.screenDensityFloat=String(c));c=sh();""!==c&&(b.client.experimentsToken=c);c=th();0<c.length&&(b.request={internalExperimentFlags:c});Sj(a,void 0,b);Tj(a,void 0,b);Uj(void 0,b);Vj(a,void 0,b);Wj(void 0,b);B("DELEGATED_SESSION_ID")&&!K("pageid_as_header_web")&&(b.user=
{onBehalfOfUser:B("DELEGATED_SESSION_ID")});a=Object;c=a.assign;for(var d=b.client,e={},f=q(Object.entries(ji(B("DEVICE","")))),g=f.next();!g.done;g=f.next()){var h=q(g.value);g=h.next().value;h=h.next().value;"cbrand"===g?e.deviceMake=h:"cmodel"===g?e.deviceModel=h:"cbr"===g?e.browserName=h:"cbrver"===g?e.browserVersion=h:"cos"===g?e.osName=h:"cosver"===g?e.osVersion=h:"cplatform"===g&&(e.platform=h)}b.client=c.call(a,d,e);return b}
function rj(a){var b=new xg,c=new qg;F(c,1,a.yb);F(c,2,a.xb);F(c,16,a.Vb);F(c,17,a.innertubeContextClientVersion);if(a.bb){var d=a.bb,e=new ng;d.coldConfigData&&F(e,1,d.coldConfigData);d.appInstallData&&F(e,6,d.appInstallData);d.coldHashData&&F(e,3,d.coldHashData);d.hotHashData&&F(e,5,d.hotHashData);G(c,62,e)}(d=x.devicePixelRatio)&&1!=d&&F(c,65,d);d=sh();""!==d&&F(c,54,d);d=th();if(0<d.length){e=new sg;for(var f=0;f<d.length;f++){var g=new lg;F(g,1,d[f].key);g.setValue(d[f].value);Uc(e,15,lg,g)}G(b,
5,e)}Sj(a,c);Tj(a,c);Uj(c);Vj(a,c);Wj(c);B("DELEGATED_SESSION_ID")&&!K("pageid_as_header_web")&&(a=new vg,F(a,3,B("DELEGATED_SESSION_ID")));a=q(Object.entries(ji(B("DEVICE",""))));for(d=a.next();!d.done;d=a.next())e=q(d.value),d=e.next().value,e=e.next().value,"cbrand"===d?F(c,12,e):"cmodel"===d?F(c,13,e):"cbr"===d?F(c,87,e):"cbrver"===d?F(c,88,e):"cos"===d?F(c,18,e):"cosver"===d?F(c,19,e):"cplatform"===d&&F(c,42,e);G(b,1,c);return b}
function Sj(a,b,c){a=a.cb;if("WEB"===a||"MWEB"===a||1===a||2===a)if(b){c=Qc(b,og,96)||new og;var d=Aj();null!==d&&F(c,3,d);G(b,96,c)}else c&&(c.client.mainAppWebInfo=null!=(d=c.client.mainAppWebInfo)?d:{},c.client.mainAppWebInfo.webDisplayMode=zj())}
function Tj(a,b,c){a=a.cb;if(("WEB_REMIX"===a||76===a)&&!K("music_web_display_mode_killswitch"))if(b){var d;c=null!=(d=Qc(b,pg,70))?d:new pg;d=Aj();null!==d&&F(c,10,d);G(b,70,c)}else if(c){var e;c.client.Ab=null!=(e=c.client.Ab)?e:{};c.client.Ab.webDisplayMode=zj()}}
function Uj(a,b){var c;if(K("web_log_memory_total_kbytes")&&(null==(c=x.navigator)?0:c.deviceMemory)){var d;c=null==(d=x.navigator)?void 0:d.deviceMemory;a?F(a,95,1E6*c):b&&(b.client.memoryTotalKbytes=""+1E6*c)}}
function Vj(a,b,c){if(a.appInstallData)if(b){var d;c=null!=(d=Qc(b,ng,62))?d:new ng;F(c,6,a.appInstallData);G(b,62,c)}else c&&(c.client.configInfo=c.client.configInfo||{},c.client.configInfo.appInstallData=a.appInstallData)}
function Wj(a,b){var c=Pj();c&&(a?F(a,61,Lj[c]):b&&(b.client.connectionType=c));K("web_log_effective_connection_type")&&(c=Qj())&&(a?F(a,94,Mj[c]):b&&(b.client.effectiveConnectionType=c))}
function Xj(a,b,c){c=void 0===c?{}:c;var d={};K("enable_web_eom_visitor_data")&&B("EOM_VISITOR_DATA")?d={"X-Goog-EOM-Visitor-Id":B("EOM_VISITOR_DATA")}:d={"X-Goog-Visitor-Id":c.visitorData||B("VISITOR_DATA","")};if(b&&b.includes("www.youtube-nocookie.com"))return d;(b=c.to||B("AUTHORIZATION"))||(a?b="Bearer "+A("gapi.auth.getToken")().so:b=Vd([]));b&&(d.Authorization=b,d["X-Goog-AuthUser"]=B("SESSION_INDEX",0),K("pageid_as_header_web")&&(d["X-Goog-PageId"]=B("DELEGATED_SESSION_ID")));return d}
;function Yj(a){a=Object.assign({},a);delete a.Authorization;var b=Vd();if(b){var c=new $e;c.update(B("INNERTUBE_API_KEY",void 0));c.update(b);a.hash=zc(c.digest(),3)}return a}
;function Zj(a){var b=new eg;(b=b.isAvailable()?a?new kg(b,a):b:null)||(a=new fg(a||"UserDataSharedStore"),b=a.isAvailable()?a:null);this.h=(a=b)?new ag(a):null;this.i=document.domain||window.location.hostname}
Zj.prototype.set=function(a,b,c,d){c=c||31104E3;this.remove(a);if(this.h)try{this.h.set(a,b,Date.now()+1E3*c);return}catch(f){}var e="";if(d)try{e=escape(wf(b))}catch(f){return}else e=escape(b);Bj(a,e,c,this.i)};
Zj.prototype.get=function(a,b){var c=void 0,d=!this.h;if(!d)try{c=this.h.get(a)}catch(e){d=!0}if(d&&(c=Cj(a))&&(c=unescape(c),b))try{c=JSON.parse(c)}catch(e){this.remove(a),c=void 0}return c};
Zj.prototype.remove=function(a){this.h&&this.h.remove(a);var b=this.i;Rd.remove(""+a,"/",void 0===b?"youtube.com":b)};var ak;function bk(){ak||(ak=new Zj("yt.innertube"));return ak}
function ck(a,b,c,d){if(d)return null;d=bk().get("nextId",!0)||1;var e=bk().get("requests",!0)||{};e[d]={method:a,request:b,authState:Yj(c),requestTime:Math.round(N())};bk().set("nextId",d+1,86400,!0);bk().set("requests",e,86400,!0);return d}
function dk(a){var b=bk().get("requests",!0)||{};delete b[a];bk().set("requests",b,86400,!0)}
function ek(a){var b=bk().get("requests",!0);if(b){for(var c in b){var d=b[c];if(!(6E4>Math.round(N())-d.requestTime)){var e=d.authState,f=Yj(Xj(!1));rb(e,f)&&(e=d.request,"requestTimeMs"in e&&(e.requestTimeMs=Math.round(N())),pj(a,d.method,e,{}));delete b[c]}}bk().set("requests",b,86400,!0)}}
;var fk=uc||vc;function gk(a){var b=Mb();return b?0<=b.toLowerCase().indexOf(a):!1}
;var hk=function(){var a;return function(){a||(a=new Zj("ytidb"));return a}}();
function ik(){var a;return null===(a=hk())||void 0===a?void 0:a.get("LAST_RESULT_ENTRY_KEY",!0)}
;var jk=[],kk,lk=!1;function mk(){var a={};for(kk=new nk(void 0===a.handleError?ok:a.handleError,void 0===a.logEvent?pk:a.logEvent);0<jk.length;)switch(a=jk.shift(),a.type){case "ERROR":kk.handleError(a.payload);break;case "EVENT":kk.logEvent(a.eventType,a.payload)}}
function qk(a){lk||(kk?kk.handleError(a):(jk.push({type:"ERROR",payload:a}),10<jk.length&&jk.shift()))}
function rk(a,b){lk||(kk?kk.logEvent(a,b):(jk.push({type:"EVENT",eventType:a,payload:b}),10<jk.length&&jk.shift()))}
;function sk(a){var b=Fa.apply(1,arguments);var c=Error.call(this,a);this.message=c.message;"stack"in c&&(this.stack=c.stack);this.args=[].concat(ha(b))}
r(sk,Error);function tk(){try{return uk(),!0}catch(a){return!1}}
function uk(){if(void 0!==B("DATASYNC_ID",void 0))return B("DATASYNC_ID",void 0);throw new sk("Datasync ID not set","unknown");}
;function vk(a){if(0<=a.indexOf(":"))throw Error("Database name cannot contain ':'");}
function wk(a){return a.substr(0,a.indexOf(":"))||a}
;var xk={},yk=(xk.AUTH_INVALID="No user identifier specified.",xk.EXPLICIT_ABORT="Transaction was explicitly aborted.",xk.IDB_NOT_SUPPORTED="IndexedDB is not supported.",xk.MISSING_INDEX="Index not created.",xk.MISSING_OBJECT_STORES="Object stores not created.",xk.DB_DELETED_BY_MISSING_OBJECT_STORES="Database is deleted because expected object stores were not created.",xk.DB_REOPENED_BY_MISSING_OBJECT_STORES="Database is reopened because expected object stores were not created.",xk.UNKNOWN_ABORT="Transaction was aborted for unknown reasons.",
xk.QUOTA_EXCEEDED="The current transaction exceeded its quota limitations.",xk.QUOTA_MAYBE_EXCEEDED="The current transaction may have failed because of exceeding quota limitations.",xk.EXECUTE_TRANSACTION_ON_CLOSED_DB="Can't start a transaction on a closed database",xk.INCOMPATIBLE_DB_VERSION="The binary is incompatible with the database version",xk),zk={},Ak=(zk.AUTH_INVALID="ERROR",zk.EXECUTE_TRANSACTION_ON_CLOSED_DB="WARNING",zk.EXPLICIT_ABORT="IGNORED",zk.IDB_NOT_SUPPORTED="ERROR",zk.MISSING_INDEX=
"WARNING",zk.MISSING_OBJECT_STORES="ERROR",zk.DB_DELETED_BY_MISSING_OBJECT_STORES="WARNING",zk.DB_REOPENED_BY_MISSING_OBJECT_STORES="WARNING",zk.QUOTA_EXCEEDED="WARNING",zk.QUOTA_MAYBE_EXCEEDED="WARNING",zk.UNKNOWN_ABORT="WARNING",zk.INCOMPATIBLE_DB_VERSION="WARNING",zk),Bk={},Ck=(Bk.AUTH_INVALID=!1,Bk.EXECUTE_TRANSACTION_ON_CLOSED_DB=!1,Bk.EXPLICIT_ABORT=!1,Bk.IDB_NOT_SUPPORTED=!1,Bk.MISSING_INDEX=!1,Bk.MISSING_OBJECT_STORES=!1,Bk.DB_DELETED_BY_MISSING_OBJECT_STORES=!1,Bk.DB_REOPENED_BY_MISSING_OBJECT_STORES=
!1,Bk.QUOTA_EXCEEDED=!1,Bk.QUOTA_MAYBE_EXCEEDED=!0,Bk.UNKNOWN_ABORT=!0,Bk.INCOMPATIBLE_DB_VERSION=!1,Bk);function Dk(a,b,c,d,e){b=void 0===b?{}:b;c=void 0===c?yk[a]:c;d=void 0===d?Ak[a]:d;e=void 0===e?Ck[a]:e;sk.call(this,c,Object.assign({name:"YtIdbKnownError",isSw:void 0===self.document,isIframe:self!==self.top,type:a},b));this.type=a;this.message=c;this.level=d;this.h=e;Object.setPrototypeOf(this,Dk.prototype)}
r(Dk,sk);function Ek(a,b){Dk.call(this,"MISSING_OBJECT_STORES",{expectedObjectStores:b,foundObjectStores:a},yk.MISSING_OBJECT_STORES);Object.setPrototypeOf(this,Ek.prototype)}
r(Ek,Dk);function Fk(a,b){var c=Error.call(this);this.message=c.message;"stack"in c&&(this.stack=c.stack);this.index=a;this.objectStore=b;Object.setPrototypeOf(this,Fk.prototype)}
r(Fk,Error);var Gk=["The database connection is closing","Can't start a transaction on a closed database","A mutation operation was attempted on a database that did not allow mutations"];
function Hk(a,b,c,d){b=wk(b);var e=a instanceof Error?a:Error("Unexpected error: "+a);if(e instanceof Dk)return e;a={objectStoreNames:c,dbName:b,dbVersion:d};if("QuotaExceededError"===e.name)return new Dk("QUOTA_EXCEEDED",a);if(wc&&"UnknownError"===e.name)return new Dk("QUOTA_MAYBE_EXCEEDED",a);if(e instanceof Fk)return new Dk("MISSING_INDEX",Object.assign(Object.assign({},a),{objectStore:e.objectStore,index:e.index}));if("InvalidStateError"===e.name&&Gk.some(function(f){return e.message.includes(f)}))return new Dk("EXECUTE_TRANSACTION_ON_CLOSED_DB",
a);
if("AbortError"===e.name)return new Dk("UNKNOWN_ABORT",a,e.message);e.args=[Object.assign(Object.assign({},a),{name:"IdbError",Cb:e.name})];e.level="WARNING";return e}
function Ik(a,b,c){var d=ik();return new Dk("IDB_NOT_SUPPORTED",{context:{caller:a,publicName:b,version:c,hasSucceededOnce:null===d||void 0===d?void 0:d.hasSucceededOnce}})}
;function Jk(a){if(!a)throw Error();throw a;}
function Kk(a){return a}
function Lk(a){this.h=a}
function Mk(a){function b(e){if("PENDING"===d.state.status){d.state={status:"REJECTED",reason:e};e=q(d.onRejected);for(var f=e.next();!f.done;f=e.next())f=f.value,f()}}
function c(e){if("PENDING"===d.state.status){d.state={status:"FULFILLED",value:e};e=q(d.h);for(var f=e.next();!f.done;f=e.next())f=f.value,f()}}
var d=this;this.state={status:"PENDING"};this.h=[];this.onRejected=[];a=a.h;try{a(c,b)}catch(e){b(e)}}
Mk.all=function(a){return new Mk(new Lk(function(b,c){var d=[],e=a.length;0===e&&b(d);for(var f={na:0};f.na<a.length;f={na:f.na},++f.na)Nk(Mk.resolve(a[f.na]).then(function(g){return function(h){d[g.na]=h;e--;0===e&&b(d)}}(f)),function(g){c(g)})}))};
Mk.resolve=function(a){return new Mk(new Lk(function(b,c){a instanceof Mk?a.then(b,c):b(a)}))};
Mk.reject=function(a){return new Mk(new Lk(function(b,c){c(a)}))};
Mk.prototype.then=function(a,b){var c=this,d=null!==a&&void 0!==a?a:Kk,e=null!==b&&void 0!==b?b:Jk;return new Mk(new Lk(function(f,g){"PENDING"===c.state.status?(c.h.push(function(){Ok(c,c,d,f,g)}),c.onRejected.push(function(){Pk(c,c,e,f,g)})):"FULFILLED"===c.state.status?Ok(c,c,d,f,g):"REJECTED"===c.state.status&&Pk(c,c,e,f,g)}))};
function Nk(a,b){a.then(void 0,b)}
function Ok(a,b,c,d,e){try{if("FULFILLED"!==a.state.status)throw Error("calling handleResolve before the promise is fulfilled.");var f=c(a.state.value);f instanceof Mk?Qk(a,b,f,d,e):d(f)}catch(g){e(g)}}
function Pk(a,b,c,d,e){try{if("REJECTED"!==a.state.status)throw Error("calling handleReject before the promise is rejected.");var f=c(a.state.reason);f instanceof Mk?Qk(a,b,f,d,e):d(f)}catch(g){e(g)}}
function Qk(a,b,c,d,e){b===c?e(new TypeError("Circular promise chain detected.")):c.then(function(f){f instanceof Mk?Qk(a,b,f,d,e):d(f)},function(f){e(f)})}
;function Rk(a,b,c){function d(){c(a.error);f()}
function e(){b(a.result);f()}
function f(){try{a.removeEventListener("success",e),a.removeEventListener("error",d)}catch(g){}}
a.addEventListener("success",e);a.addEventListener("error",d)}
function Sk(a){return new Promise(function(b,c){Rk(a,b,c)})}
function Tk(a){return new Mk(new Lk(function(b,c){Rk(a,b,c)}))}
;function Uk(a,b){return new Mk(new Lk(function(c,d){function e(){var f=a?b(a):null;f?f.then(function(g){a=g;e()},d):c()}
e()}))}
;function Vk(a,b){this.h=a;this.options=b;this.transactionCount=0;this.j=Math.round(N());this.i=!1}
m=Vk.prototype;m.add=function(a,b,c){return Wk(this,[a],{mode:"readwrite",O:!0},function(d){return d.objectStore(a).add(b,c)})};
m.clear=function(a){return Wk(this,[a],{mode:"readwrite",O:!0},function(b){return b.objectStore(a).clear()})};
m.close=function(){var a;this.h.close();(null===(a=this.options)||void 0===a?0:a.closed)&&this.options.closed()};
m.count=function(a,b){return Wk(this,[a],{mode:"readonly",O:!0},function(c){return c.objectStore(a).count(b)})};
function Xk(a,b,c){a=a.h.createObjectStore(b,c);return new Yk(a)}
m.delete=function(a,b){return Wk(this,[a],{mode:"readwrite",O:!0},function(c){return c.objectStore(a).delete(b)})};
m.get=function(a,b){return Wk(this,[a],{mode:"readonly",O:!0},function(c){return c.objectStore(a).get(b)})};
function Zk(a,b){return Wk(a,["LogsRequestsStore"],{mode:"readwrite",O:!0},function(c){c=c.objectStore("LogsRequestsStore");return Tk(c.h.put(b,void 0))})}
m.objectStoreNames=function(){return Array.from(this.h.objectStoreNames)};
function Wk(a,b,c,d){var e,f,g,h,k,l,n,t,z,u,D,E;return w(function(L){switch(L.h){case 1:var P={mode:"readonly",O:!1,tag:"IDB_TRANSACTION_TAG_UNKNOWN"};"string"===typeof c?P.mode=c:Object.assign(P,c);e=P;a.transactionCount++;f=e.O?3:1;g=0;case 2:if(h){L.s(3);break}g++;k=Math.round(N());ta(L,4);l=a.h.transaction(b,e.mode);P=new $k(l);P=al(P,d);return v(L,P,6);case 6:return n=L.i,t=Math.round(N()),bl(a,k,t,g,void 0,b.join(),e),L.return(n);case 4:z=wa(L);u=Math.round(N());D=Hk(z,a.h.name,b.join(),a.h.version);
if((E=D instanceof Dk&&!D.h)||g>=f)bl(a,k,u,g,D,b.join(),e),h=D;L.s(2);break;case 3:return L.return(Promise.reject(h))}})}
function bl(a,b,c,d,e,f,g){b=c-b;e?(e instanceof Dk&&("QUOTA_EXCEEDED"===e.type||"QUOTA_MAYBE_EXCEEDED"===e.type)&&rk("QUOTA_EXCEEDED",{dbName:wk(a.h.name),objectStoreNames:f,transactionCount:a.transactionCount,transactionMode:g.mode}),e instanceof Dk&&"UNKNOWN_ABORT"===e.type&&(c-=a.j,0>c&&c>=Math.pow(2,31)&&(c=0),rk("TRANSACTION_UNEXPECTEDLY_ABORTED",{objectStoreNames:f,transactionDuration:b,transactionCount:a.transactionCount,dbDuration:c}),a.i=!0),cl(a,!1,d,f,b,g.tag),qk(e)):cl(a,!0,d,f,b,g.tag)}
function cl(a,b,c,d,e,f){rk("TRANSACTION_ENDED",{objectStoreNames:d,connectionHasUnknownAbortedTransaction:a.i,duration:e,isSuccessful:b,tryCount:c,tag:void 0===f?"IDB_TRANSACTION_TAG_UNKNOWN":f})}
m.getName=function(){return this.h.name};
function Yk(a){this.h=a}
m=Yk.prototype;m.add=function(a,b){return Tk(this.h.add(a,b))};
m.autoIncrement=function(){return this.h.autoIncrement};
m.clear=function(){return Tk(this.h.clear()).then(function(){})};
m.count=function(a){return Tk(this.h.count(a))};
function dl(a,b){return el(a,{query:b},function(c){return c.delete().then(function(){return c.continue()})}).then(function(){})}
m.delete=function(a){return a instanceof IDBKeyRange?dl(this,a):Tk(this.h.delete(a))};
m.get=function(a){return Tk(this.h.get(a))};
m.index=function(a){try{return new fl(this.h.index(a))}catch(b){if(b instanceof Error&&"NotFoundError"===b.name)throw new Fk(a,this.h.name);throw b;}};
m.getName=function(){return this.h.name};
m.keyPath=function(){return this.h.keyPath};
function el(a,b,c){a=a.h.openCursor(b.query,b.direction);return gl(a).then(function(d){return Uk(d,c)})}
function $k(a){var b=this;this.h=a;this.j=new Map;this.i=!1;this.done=new Promise(function(c,d){b.h.addEventListener("complete",function(){c()});
b.h.addEventListener("error",function(e){e.currentTarget===e.target&&d(b.h.error)});
b.h.addEventListener("abort",function(){var e=b.h.error;if(e)d(e);else if(!b.i){e=Dk;for(var f=b.h.objectStoreNames,g=[],h=0;h<f.length;h++){var k=f.item(h);if(null===k)throw Error("Invariant: item in DOMStringList is null");g.push(k)}e=new e("UNKNOWN_ABORT",{objectStoreNames:g.join(),dbName:b.h.db.name,mode:b.h.mode});d(e)}})})}
function al(a,b){var c=new Promise(function(d,e){try{Nk(b(a).then(function(f){d(f)}),e)}catch(f){e(f),a.abort()}});
return Promise.all([c,a.done]).then(function(d){return q(d).next().value})}
$k.prototype.abort=function(){this.h.abort();this.i=!0;throw new Dk("EXPLICIT_ABORT");};
$k.prototype.objectStore=function(a){a=this.h.objectStore(a);var b=this.j.get(a);b||(b=new Yk(a),this.j.set(a,b));return b};
function fl(a){this.h=a}
m=fl.prototype;m.count=function(a){return Tk(this.h.count(a))};
m.delete=function(a){return hl(this,{query:a},function(b){return b.delete().then(function(){return b.continue()})})};
m.get=function(a){return Tk(this.h.get(a))};
m.getKey=function(a){return Tk(this.h.getKey(a))};
m.keyPath=function(){return this.h.keyPath};
m.unique=function(){return this.h.unique};
function hl(a,b,c){a=a.h.openCursor(void 0===b.query?null:b.query,void 0===b.direction?"next":b.direction);return gl(a).then(function(d){return Uk(d,c)})}
function il(a,b){this.request=a;this.cursor=b}
function gl(a){return Tk(a).then(function(b){return b?new il(a,b):null})}
m=il.prototype;m.advance=function(a){this.cursor.advance(a);return gl(this.request)};
m.continue=function(a){this.cursor.continue(a);return gl(this.request)};
m.delete=function(){return Tk(this.cursor.delete()).then(function(){})};
m.getKey=function(){return this.cursor.key};
m.getValue=function(){return this.cursor.value};
m.update=function(a){return Tk(this.cursor.update(a))};function jl(a,b,c){return new Promise(function(d,e){function f(){z||(z=new Vk(g.result,{closed:t}));return z}
var g=void 0!==b?self.indexedDB.open(a,b):self.indexedDB.open(a);var h=c.blocked,k=c.blocking,l=c.yc,n=c.upgrade,t=c.closed,z;g.addEventListener("upgradeneeded",function(u){try{if(null===u.newVersion)throw Error("Invariant: newVersion on IDbVersionChangeEvent is null");if(null===g.transaction)throw Error("Invariant: transaction on IDbOpenDbRequest is null");u.dataLoss&&"none"!==u.dataLoss&&rk("IDB_DATA_CORRUPTED",{reason:u.dataLossMessage||"unknown reason",dbName:wk(a)});var D=f(),E=new $k(g.transaction);
n&&n(D,function(L){return u.oldVersion<L&&u.newVersion>=L},E);
E.done.catch(function(L){e(L)})}catch(L){e(L)}});
g.addEventListener("success",function(){var u=g.result;k&&u.addEventListener("versionchange",function(){k(f())});
u.addEventListener("close",function(){rk("IDB_UNEXPECTEDLY_CLOSED",{dbName:wk(a),dbVersion:u.version});l&&l()});
d(f())});
g.addEventListener("error",function(){e(g.error)});
h&&g.addEventListener("blocked",function(){h()})})}
function kl(a,b,c){c=void 0===c?{}:c;return jl(a,b,c)}
function ll(a,b){b=void 0===b?{}:b;var c,d,e,f;return w(function(g){if(1==g.h)return ta(g,2),c=self.indexedDB.deleteDatabase(a),d=b,(e=d.blocked)&&c.addEventListener("blocked",function(){e()}),v(g,Sk(c),4);
if(2!=g.h)return va(g,0);f=wa(g);throw Hk(f,a,"",-1);})}
;function ml(a){return new Promise(function(b){Rh(function(){b()},a)})}
function nl(a,b){this.name=a;this.options=b;this.l=!0;this.m=this.o=0;this.i=500}
nl.prototype.j=function(a,b,c){c=void 0===c?{}:c;return kl(a,b,c)};
nl.prototype.delete=function(a){a=void 0===a?{}:a;return ll(this.name,a)};
function ol(a,b){return new Dk("INCOMPATIBLE_DB_VERSION",{dbName:a.name,oldVersion:a.options.version,newVersion:b})}
function pl(a,b){if(!b)throw Ik("openWithToken",wk(a.name));return a.open()}
nl.prototype.open=function(){function a(){var f,g,h,k,l,n,t,z,u,D;return w(function(E){switch(E.h){case 1:return h=null!==(f=Error().stack)&&void 0!==f?f:"",ta(E,2),v(E,c.j(c.name,c.options.version,e),4);case 4:k=E.i;for(var L=c.options,P=[],S=q(Object.keys(L.ya)),W=S.next();!W.done;W=S.next()){W=W.value;var Z=L.ya[W],Cd=void 0===Z.hc?Number.MAX_VALUE:Z.hc;!(k.h.version>=Z.Wa)||k.h.version>=Cd||k.h.objectStoreNames.contains(W)||P.push(W)}l=P;if(0===l.length){E.s(5);break}n=Object.keys(c.options.ya);
t=k.objectStoreNames();if(c.m<rh("ytidb_reopen_db_retries",0))return c.m++,k.close(),qk(new Dk("DB_REOPENED_BY_MISSING_OBJECT_STORES",{dbName:c.name,expectedObjectStores:n,foundObjectStores:t})),E.return(a());if(!(c.o<rh("ytidb_remake_db_retries",1))){E.s(6);break}c.o++;if(!K("ytidb_remake_db_enable_backoff_delay")){E.s(7);break}return v(E,ml(c.i),8);case 8:c.i*=2;case 7:return v(E,c.delete(),9);case 9:return qk(new Dk("DB_DELETED_BY_MISSING_OBJECT_STORES",{dbName:c.name,expectedObjectStores:n,foundObjectStores:t})),
E.return(a());case 6:throw new Ek(t,n);case 5:return E.return(k);case 2:z=wa(E);if(z instanceof DOMException?"VersionError"!==z.name:"DOMError"in self&&z instanceof DOMError?"VersionError"!==z.name:!(z instanceof Object&&"message"in z)||"An attempt was made to open a database using a lower version than the existing version."!==z.message){E.s(10);break}return v(E,c.j(c.name,void 0,Object.assign(Object.assign({},e),{upgrade:void 0})),11);case 11:u=E.i;D=u.h.version;if(void 0!==c.options.version&&D>
c.options.version+1)throw u.close(),c.l=!1,ol(c,D);return E.return(u);case 10:throw b(),z instanceof Error&&!K("ytidb_async_stack_killswitch")&&(z.stack=z.stack+"\n"+h.substring(h.indexOf("\n")+1)),Hk(z,c.name,"",null!==(g=c.options.version)&&void 0!==g?g:-1);}})}
function b(){c.h===d&&(c.h=void 0)}
var c=this;if(!this.l)throw ol(this);if(this.h)return this.h;var d,e={blocking:function(f){f.close()},
closed:b,yc:b,upgrade:this.options.upgrade};return this.h=d=a()};var ql=new nl("YtIdbMeta",{ya:{databases:{Wa:1}},upgrade:function(a,b){b(1)&&Xk(a,"databases",{keyPath:"actualName"})}});
function rl(a,b){var c;return w(function(d){if(1==d.h)return v(d,pl(ql,b),2);c=d.i;return d.return(Wk(c,["databases"],{O:!0,mode:"readwrite"},function(e){var f=e.objectStore("databases");return f.get(a.actualName).then(function(g){if(g?a.actualName!==g.actualName||a.publicName!==g.publicName||a.userIdentifier!==g.userIdentifier:1)return Tk(f.h.put(a,void 0)).then(function(){})})}))})}
function sl(a,b){var c;return w(function(d){if(1==d.h)return a?v(d,pl(ql,b),2):d.return();c=d.i;return d.return(c.delete("databases",a))})}
function tl(a,b){var c,d;return w(function(e){return 1==e.h?(c=[],v(e,pl(ql,b),2)):3!=e.h?(d=e.i,v(e,Wk(d,["databases"],{O:!0,mode:"readonly"},function(f){c.length=0;return el(f.objectStore("databases"),{},function(g){a(g.getValue())&&c.push(g.getValue());return g.continue()})}),3)):e.return(c)})}
function ul(a){return tl(function(b){return"LogsDatabaseV2"===b.publicName&&void 0!==b.userIdentifier},a)}
function vl(a,b){return tl(function(c){return void 0!==c.userIdentifier&&!a.includes(c.userIdentifier)},b)}
;var wl,xl=new function(){}(new function(){});
function yl(){var a,b,c;return w(function(d){switch(d.h){case 1:a=ik();if(null===a||void 0===a?0:a.hasSucceededOnce)return d.return(!0);var e;if(e=fk)e=/WebKit\/([0-9]+)/.exec(Mb()),e=!!(e&&600<=parseInt(e[1],10));e&&(e=/WebKit\/([0-9]+)/.exec(Mb()),e=!(e&&602<=parseInt(e[1],10)));if(e||hc)return d.return(!1);try{if(b=self,!(b.indexedDB&&b.IDBIndex&&b.IDBKeyRange&&b.IDBObjectStore))return d.return(!1)}catch(f){return d.return(!1)}if(!("IDBTransaction"in self&&"objectStoreNames"in IDBTransaction.prototype))return d.return(!1);
ta(d,2);c={actualName:"yt-idb-test-do-not-use",publicName:"yt-idb-test-do-not-use",userIdentifier:void 0};return v(d,rl(c,xl),4);case 4:return v(d,sl("yt-idb-test-do-not-use",xl),5);case 5:return d.return(!0);case 2:return wa(d),d.return(!1)}})}
function zl(){if(void 0!==wl)return wl;lk=!0;return wl=yl().then(function(a){lk=!1;var b,c;null!==(b=hk())&&void 0!==b&&b.h&&(b=ik(),b={hasSucceededOnce:(null===b||void 0===b?void 0:b.hasSucceededOnce)||a},null===(c=hk())||void 0===c?void 0:c.set("LAST_RESULT_ENTRY_KEY",b,2592E3,!0));return a})}
function Al(){return A("ytglobal.idbToken_")||void 0}
function Bl(){var a=Al();return a?Promise.resolve(a):zl().then(function(b){(b=b?xl:void 0)&&y("ytglobal.idbToken_",b,void 0);return b})}
;var Cl=0;function Dl(a){Cl||(Cl=Vh.N(function(){var b,c,d,e,f;return w(function(g){switch(g.h){case 1:return v(g,Bl(),2);case 2:b=g.i;if(!b)return g.return();c=!0;ta(g,3);return v(g,vl(a,b),5);case 5:d=g.i;if(!d.length){c=!1;g.s(6);break}e=d[0];return v(g,ll(e.actualName),7);case 7:return v(g,sl(e.actualName,b),6);case 6:va(g,4);break;case 3:f=wa(g),qk(f),c=!1;case 4:Vh.U(Cl),Cl=0,c&&Dl(a),g.h=0}})}))}
new jd;function El(a){if(!tk())throw a=new Dk("AUTH_INVALID",{dbName:a}),qk(a),a;var b=uk();return{actualName:a+":"+b,publicName:a,userIdentifier:b}}
function Fl(a,b,c,d){var e,f,g,h,k,l;return w(function(n){switch(n.h){case 1:return f=null!==(e=Error().stack)&&void 0!==e?e:"",v(n,Bl(),2);case 2:g=n.i;if(!g)throw h=Ik("openDbImpl",a,b),K("ytidb_async_stack_killswitch")||(h.stack=h.stack+"\n"+f.substring(f.indexOf("\n")+1)),qk(h),h;vk(a);k=c?{actualName:a,publicName:a,userIdentifier:void 0}:El(a);ta(n,3);return v(n,rl(k,g),5);case 5:return v(n,kl(k.actualName,b,d),6);case 6:return n.return(n.i);case 3:return l=wa(n),ta(n,7),v(n,sl(k.actualName,
g),9);case 9:va(n,8);break;case 7:wa(n);case 8:throw l;}})}
function Gl(a,b,c){c=void 0===c?{}:c;return Fl(a,b,!1,c)}
function Hl(a,b,c){c=void 0===c?{}:c;return Fl(a,b,!0,c)}
function Il(a,b){b=void 0===b?{}:b;var c,d;return w(function(e){if(1==e.h)return v(e,Bl(),2);if(3!=e.h){c=e.i;if(!c)return e.return();vk(a);d=El(a);return v(e,ll(d.actualName,b),3)}return v(e,sl(d.actualName,c),0)})}
function Jl(a,b,c){a=a.map(function(d){return w(function(e){return 1==e.h?v(e,ll(d.actualName,b),2):v(e,sl(d.actualName,c),0)})});
return Promise.all(a).then(function(){})}
function Kl(){var a=void 0===a?{}:a;var b,c;return w(function(d){if(1==d.h)return v(d,Bl(),2);if(3!=d.h){b=d.i;if(!b)return d.return();vk("LogsDatabaseV2");return v(d,ul(b),3)}c=d.i;return v(d,Jl(c,a,b),0)})}
function Ll(a,b){b=void 0===b?{}:b;var c;return w(function(d){if(1==d.h)return v(d,Bl(),2);if(3!=d.h){c=d.i;if(!c)return d.return();vk(a);return v(d,ll(a,b),3)}return v(d,sl(a,c),0)})}
;function Ml(a){var b,c,d,e,f,g,h,k;this.h=!1;this.potentialEsfErrorCounter=this.i=0;this.handleError=function(){};
this.ra=function(){};
this.now=Date.now;this.xa=!1;this.Jb=null!==(b=a.Jb)&&void 0!==b?b:100;this.Hb=null!==(c=a.Hb)&&void 0!==c?c:1;this.Fb=null!==(d=a.Fb)&&void 0!==d?d:2592E6;this.Db=null!==(e=a.Db)&&void 0!==e?e:12E4;this.Gb=null!==(f=a.Gb)&&void 0!==f?f:5E3;this.v=null!==(g=a.v)&&void 0!==g?g:void 0;this.La=!!a.La;this.Ka=null!==(h=a.Ka)&&void 0!==h?h:.1;this.Ra=null!==(k=a.Ra)&&void 0!==k?k:10;a.handleError&&(this.handleError=a.handleError);a.ra&&(this.ra=a.ra);a.xa&&(this.xa=a.xa);this.C=a.C;this.V=a.V;this.K=a.K;
this.J=a.J;this.ea=a.ea;this.ib=a.ib;this.hb=a.hb;this.v&&(!this.C||this.C("networkless_logging"))&&Nl(this)}
function Nl(a){a.v&&!a.xa&&(a.h=!0,a.La&&Math.random()<=a.Ka&&a.K.Pb(a.v),Ol(a),a.J.H()&&a.Ba(),a.C&&!a.C("use_new_nwl_initialization")&&(a.J.W(a.ib,a.Ba.bind(a)),a.J.W(a.hb,a.nb.bind(a))))}
m=Ml.prototype;m.writeThenSend=function(a,b){var c=this;b=void 0===b?{}:b;if(this.v&&this.h){var d={url:a,options:b,timestamp:this.now(),status:"NEW",sendCount:0};this.K.set(d,this.v).then(function(e){d.id=e;c.J.H()&&Pl(c,d)}).catch(function(e){Pl(c,d);
Ql(c,e)})}else this.ea(a,b)};
m.sendThenWrite=function(a,b,c){var d=this;b=void 0===b?{}:b;if(this.v&&this.h){var e={url:a,options:b,timestamp:this.now(),status:"NEW",sendCount:0};this.C&&this.C("nwl_skip_retry")&&(e.skipRetry=c);if(this.J.H()||this.C&&this.C("nwl_aggressive_send_then_write")&&!e.skipRetry){if(!e.skipRetry){var f=b.onError?b.onError:function(){};
b.onError=function(g,h){return w(function(k){if(1==k.h)return v(k,d.K.set(e,d.v).catch(function(l){Ql(d,l)}),2);
f(g,h);k.h=0})}}this.ea(a,b,e.skipRetry)}else this.K.set(e,this.v).catch(function(g){d.ea(a,b,e.skipRetry);
Ql(d,g)})}else this.ea(a,b,this.C&&this.C("nwl_skip_retry")&&c)};
m.sendAndWrite=function(a,b){var c=this;b=void 0===b?{}:b;if(this.v&&this.h){var d={url:a,options:b,timestamp:this.now(),status:"NEW",sendCount:0},e=!1,f=b.onSuccess?b.onSuccess:function(){};
d.options.onSuccess=function(g,h){void 0!==d.id?c.K.qa(d.id,c.v):e=!0;c.J.ca&&c.C&&c.C("vss_network_hint")&&c.J.ca(!0);f(g,h)};
this.ea(d.url,d.options);this.K.set(d,this.v).then(function(g){d.id=g;e&&c.K.qa(d.id,c.v)}).catch(function(g){Ql(c,g)})}else this.ea(a,b)};
m.Ba=function(){var a=this;if(!this.v)throw Ik("throttleSend");this.i||(this.i=this.V.N(function(){var b;return w(function(c){if(1==c.h)return v(c,a.K.wb("NEW",a.v),2);if(3!=c.h)return b=c.i,b?v(c,Pl(a,b),3):(a.nb(),c.return());a.i&&(a.i=0,a.Ba());c.h=0})},this.Jb))};
m.nb=function(){this.V.U(this.i);this.i=0};
function Pl(a,b){var c,d;return w(function(e){switch(e.h){case 1:if(!a.v)throw c=Ik("immediateSend"),c;if(void 0===b.id){e.s(2);break}return v(e,a.K.Zb(b.id,a.v),3);case 3:(d=e.i)?b=d:a.ra(Error("The request cannot be found in the database."));case 2:if(Rl(a,b,a.Fb)){e.s(4);break}a.ra(Error("Networkless Logging: Stored logs request expired age limit"));if(void 0===b.id){e.s(5);break}return v(e,a.K.qa(b.id,a.v),5);case 5:return e.return();case 4:b.skipRetry||(b=Sl(a,b));if(!b){e.s(0);break}if(!b.skipRetry||
void 0===b.id){e.s(8);break}return v(e,a.K.qa(b.id,a.v),8);case 8:a.ea(b.url,b.options,!!b.skipRetry),e.h=0}})}
function Sl(a,b){if(!a.v)throw Ik("updateRequestHandlers");var c=b.options.onError?b.options.onError:function(){};
b.options.onError=function(e,f){var g;return w(function(h){switch(h.h){case 1:g=Tl(f);if(!(a.C&&a.C("nwl_consider_error_code")&&g||a.C&&!a.C("nwl_consider_error_code")&&a.potentialEsfErrorCounter<=a.Ra)){h.s(2);break}if(!a.J.fa){h.s(3);break}return v(h,a.J.fa(),3);case 3:if(a.J.H()){h.s(2);break}c(e,f);if(!a.C||!a.C("nwl_consider_error_code")||void 0===(null===b||void 0===b?void 0:b.id)){h.s(6);break}return v(h,a.K.jb(b.id,a.v,!1),6);case 6:return h.return();case 2:if(a.C&&a.C("nwl_consider_error_code")&&
!g&&a.potentialEsfErrorCounter>a.Ra)return h.return();a.potentialEsfErrorCounter++;if(void 0===(null===b||void 0===b?void 0:b.id)){h.s(8);break}return b.sendCount<a.Hb?v(h,a.K.jb(b.id,a.v),12):v(h,a.K.qa(b.id,a.v),8);case 12:a.V.N(function(){a.J.H()&&a.Ba()},a.Gb);
case 8:c(e,f),h.h=0}})};
var d=b.options.onSuccess?b.options.onSuccess:function(){};
b.options.onSuccess=function(e,f){return w(function(g){if(1==g.h)return void 0===(null===b||void 0===b?void 0:b.id)?g.s(2):v(g,a.K.qa(b.id,a.v),2);a.J.ca&&a.C&&a.C("vss_network_hint")&&a.J.ca(!0);d(e,f);g.h=0})};
return b}
function Rl(a,b,c){b=b.timestamp;return a.now()-b>=c?!1:!0}
function Ol(a){if(!a.v)throw Ik("retryQueuedRequests");a.K.wb("QUEUED",a.v).then(function(b){b&&!Rl(a,b,a.Db)?a.V.N(function(){return w(function(c){if(1==c.h)return void 0===b.id?c.s(2):v(c,a.K.jb(b.id,a.v),2);Ol(a);c.h=0})}):a.J.H()&&a.Ba()})}
function Ql(a,b){a.Kb&&!a.J.H()?a.Kb(b):a.handleError(b)}
function Tl(a){var b;return(a=null===(b=null===a||void 0===a?void 0:a.error)||void 0===b?void 0:b.code)&&400<=a&&599>=a?!1:!0}
;function Ul(a,b){this.version=a;this.args=b}
;function Vl(a,b){this.topic=a;this.h=b}
Vl.prototype.toString=function(){return this.topic};var Wl=A("ytPubsub2Pubsub2Instance")||new J;J.prototype.subscribe=J.prototype.subscribe;J.prototype.unsubscribeByKey=J.prototype.va;J.prototype.publish=J.prototype.ka;J.prototype.clear=J.prototype.clear;y("ytPubsub2Pubsub2Instance",Wl,void 0);var Xl=A("ytPubsub2Pubsub2SubscribedKeys")||{};y("ytPubsub2Pubsub2SubscribedKeys",Xl,void 0);var Yl=A("ytPubsub2Pubsub2TopicToKeys")||{};y("ytPubsub2Pubsub2TopicToKeys",Yl,void 0);var Zl=A("ytPubsub2Pubsub2IsAsync")||{};y("ytPubsub2Pubsub2IsAsync",Zl,void 0);
y("ytPubsub2Pubsub2SkipSubKey",null,void 0);function $l(a,b){var c=am();c&&c.publish.call(c,a.toString(),a,b)}
function bm(a){var b=cm,c=am();if(!c)return 0;var d=c.subscribe(b.toString(),function(e,f){var g=A("ytPubsub2Pubsub2SkipSubKey");g&&g==d||(g=function(){if(Xl[d])try{if(f&&b instanceof Vl&&b!=e)try{var h=b.h,k=f;if(!k.args||!k.version)throw Error("yt.pubsub2.Data.deserialize(): serializedData is incomplete.");try{if(!h.ja){var l=new h;h.ja=l.version}var n=h.ja}catch(L){}if(!n||k.version!=n)throw Error("yt.pubsub2.Data.deserialize(): serializedData version is incompatible.");try{n=Reflect;var t=n.construct;
var z=k.args,u=z.length;if(0<u){var D=Array(u);for(k=0;k<u;k++)D[k]=z[k];var E=D}else E=[];f=t.call(n,h,E)}catch(L){throw L.message="yt.pubsub2.Data.deserialize(): "+L.message,L;}}catch(L){throw L.message="yt.pubsub2.pubsub2 cross-binary conversion error for "+b.toString()+": "+L.message,L;}a.call(window,f)}catch(L){M(L)}},Zl[b.toString()]?A("yt.scheduler.instance")?Vh.N(g):Lh(g,0):g())});
Xl[d]=!0;Yl[b.toString()]||(Yl[b.toString()]=[]);Yl[b.toString()].push(d);return d}
function dm(){var a=em,b=bm(function(c){a.apply(void 0,arguments);fm(b)});
return b}
function fm(a){var b=am();b&&("number"===typeof a&&(a=[a]),db(a,function(c){b.unsubscribeByKey(c);delete Xl[c]}))}
function am(){return A("ytPubsub2Pubsub2Instance")}
;function gm(a,b){nl.call(this,a,b);this.options=b;vk(a)}
r(gm,nl);function hm(a,b){var c;return function(){c||(c=new gm(a,b));return c}}
gm.prototype.j=function(a,b,c){c=void 0===c?{}:c;return(this.options.kb?Hl:Gl)(a,b,Object.assign({},c))};
gm.prototype.delete=function(a){a=void 0===a?{}:a;return(this.options.kb?Ll:Il)(this.name,a)};
function im(a,b){return hm(a,b)}
;var jm;
function km(){if(jm)return jm();var a={};jm=im("LogsDatabaseV2",{ya:(a.LogsRequestsStore={Wa:2},a),kb:!1,upgrade:function(b,c,d){c(2)&&Xk(b,"LogsRequestsStore",{keyPath:"id",autoIncrement:!0});c(3);c(5)&&(d=d.objectStore("LogsRequestsStore"),d.h.indexNames.contains("newRequest")&&d.h.deleteIndex("newRequest"),d.h.createIndex("newRequestV2",["status","interface","timestamp"],{unique:!1}));c(7)&&b.h.objectStoreNames.contains("sapisid")&&b.h.deleteObjectStore("sapisid");c(9)&&b.h.objectStoreNames.contains("SWHealthLog")&&b.h.deleteObjectStore("SWHealthLog")},
version:9});return jm()}
;function lm(a){return pl(km(),a)}
function mm(a,b){var c,d,e,f;return w(function(g){if(1==g.h)return c={startTime:N(),transactionType:"YT_IDB_TRANSACTION_TYPE_WRITE"},v(g,lm(b),2);if(3!=g.h)return d=g.i,e=Object.assign(Object.assign({},a),{options:JSON.parse(JSON.stringify(a.options)),interface:B("INNERTUBE_CONTEXT_CLIENT_NAME",0)}),v(g,Zk(d,e),3);f=g.i;c.zc=N();nm(c);return g.return(f)})}
function om(a,b){var c,d,e,f,g,h,k;return w(function(l){if(1==l.h)return c={startTime:N(),transactionType:"YT_IDB_TRANSACTION_TYPE_READ"},v(l,lm(b),2);if(3!=l.h)return d=l.i,e=B("INNERTUBE_CONTEXT_CLIENT_NAME",0),f=[a,e,0],g=[a,e,N()],h=IDBKeyRange.bound(f,g),k=void 0,v(l,Wk(d,["LogsRequestsStore"],{mode:"readwrite",O:!0},function(n){return hl(n.objectStore("LogsRequestsStore").index("newRequestV2"),{query:h,direction:"prev"},function(t){t.getValue()&&(k=t.getValue(),"NEW"===a&&(k.status="QUEUED",
t.update(k)))})}),3);
c.zc=N();nm(c);return l.return(k)})}
function pm(a,b){var c;return w(function(d){if(1==d.h)return v(d,lm(b),2);c=d.i;return d.return(Wk(c,["LogsRequestsStore"],{mode:"readwrite",O:!0},function(e){var f=e.objectStore("LogsRequestsStore");return f.get(a).then(function(g){if(g)return g.status="QUEUED",Tk(f.h.put(g,void 0)).then(function(){return g})})}))})}
function qm(a,b,c){c=void 0===c?!0:c;var d;return w(function(e){if(1==e.h)return v(e,lm(b),2);d=e.i;return e.return(Wk(d,["LogsRequestsStore"],{mode:"readwrite",O:!0},function(f){var g=f.objectStore("LogsRequestsStore");return g.get(a).then(function(h){return h?(h.status="NEW",c&&(h.sendCount+=1),Tk(g.h.put(h,void 0)).then(function(){return h})):Mk.resolve(void 0)})}))})}
function rm(a,b){var c;return w(function(d){if(1==d.h)return v(d,lm(b),2);c=d.i;return d.return(c.delete("LogsRequestsStore",a))})}
function sm(a){var b,c;return w(function(d){if(1==d.h)return v(d,lm(a),2);b=d.i;c=N()-2592E6;return v(d,Wk(b,["LogsRequestsStore"],{mode:"readwrite",O:!0},function(e){return el(e.objectStore("LogsRequestsStore"),{},function(f){if(f.getValue().timestamp<=c)return f.delete().then(function(){return f.continue()})})}),0)})}
function tm(){return w(function(a){return v(a,Kl(),0)})}
function nm(a){K("nwl_csi_killswitch")||.01>=Math.random()&&$l("nwl_transaction_latency_payload",a)}
;var um={},vm=im("ServiceWorkerLogsDatabase",{ya:(um.SWHealthLog={Wa:1},um),kb:!0,upgrade:function(a,b){b(1)&&Xk(a,"SWHealthLog",{keyPath:"id",autoIncrement:!0}).h.createIndex("swHealthNewRequest",["interface","timestamp"],{unique:!1})},
version:1});function wm(a){return pl(vm(),a)}
function xm(a){var b,c;return w(function(d){if(1==d.h)return v(d,wm(a),2);b=d.i;c=N()-2592E6;return v(d,Wk(b,["SWHealthLog"],{mode:"readwrite",O:!0},function(e){return el(e.objectStore("SWHealthLog"),{},function(f){if(f.getValue().timestamp<=c)return f.delete().then(function(){return f.continue()})})}),0)})}
function ym(a){var b;return w(function(c){if(1==c.h)return v(c,wm(a),2);b=c.i;return v(c,b.clear("SWHealthLog"),0)})}
;var zm={},Am=0;
function Bm(a){var b=void 0===b?"":b;if(a)if(b)Ai(a,void 0,"POST",b,void 0);else if(B("USE_NET_AJAX_FOR_PING_TRANSPORT",!1))Ai(a,void 0,"GET","",void 0);else{b:{try{var c=new ab({url:a});if(c.j&&c.i||c.l){var d=Xb(a.match(Wb)[5]||null),e;if(!(e=!d||!d.endsWith("/aclk"))){var f=a.search(cc);d:{for(b=0;0<=(b=a.indexOf("ri",b))&&b<f;){var g=a.charCodeAt(b-1);if(38==g||63==g){var h=a.charCodeAt(b+2);if(!h||61==h||38==h||35==h){var k=b;break d}}b+=3}k=-1}if(0>k)var l=null;else{var n=a.indexOf("&",k);if(0>
n||n>f)n=f;k+=3;l=decodeURIComponent(a.substr(k,n-k).replace(/\+/g," "))}e="1"!==l}var t=!e;break b}}catch(u){}t=!1}if(t){b:{try{if(window.navigator&&window.navigator.sendBeacon&&window.navigator.sendBeacon(a,"")){var z=!0;break b}}catch(u){}z=!1}t=z?!0:!1}else t=!1;t||Cm(a)}}
function Cm(a){var b=new Image,c=""+Am++;zm[c]=b;b.onload=b.onerror=function(){delete zm[c]};
b.src=a}
;function Dm(){this.h=new Map;this.i=!1}
function Em(){if(!Dm.h){var a=A("yt.networkRequestMonitor.instance")||new Dm;y("yt.networkRequestMonitor.instance",a,void 0);Dm.h=a}return Dm.h}
Dm.prototype.requestComplete=function(a,b){b&&(this.i=!0);a=this.removeParams(a);this.h.get(a)||this.h.set(a,b)};
Dm.prototype.isEndpointCFR=function(a){a=this.removeParams(a);return(a=this.h.get(a))?!1:!1===a&&this.i?!0:null};
Dm.prototype.removeParams=function(a){return a.split("?")[0]};
Dm.prototype.removeParams=Dm.prototype.removeParams;Dm.prototype.isEndpointCFR=Dm.prototype.isEndpointCFR;Dm.prototype.requestComplete=Dm.prototype.requestComplete;Dm.getInstance=Em;var Fm;function Gm(){Fm||(Fm=new Zj("yt.offline"));return Fm}
function Hm(a){if(K("offline_error_handling")){var b=Gm().get("errors",!0)||{};b[a.message]={name:a.name,stack:a.stack};a.level&&(b[a.message].level=a.level);Gm().set("errors",b,2592E3,!0)}}
function Im(){if(K("offline_error_handling")){var a=Gm().get("errors",!0);if(a){for(var b in a)if(a[b]){var c=new sk(b,"sent via offline_errors");c.name=a[b].name;c.stack=a[b].stack;c.level=a[b].level;M(c)}Gm().set("errors",{},2592E3,!0)}}}
;var Jm=rh("network_polling_interval",3E4);function O(){Be.call(this);this.M=0;this.S=this.m=!1;this.l=this.ab();K("use_shared_nsm")?(Ee.h||(Ee.h=new Ee(Vh)),this.j=Ee.h):(Km(this),Lm(this))}
r(O,Be);function Mm(){if(!O.h){var a=A("yt.networkStatusManager.instance")||new O;y("yt.networkStatusManager.instance",a,void 0);O.h=a}return O.h}
m=O.prototype;m.H=function(){var a;return K("use_shared_nsm")&&this.j?null===(a=this.j)||void 0===a?void 0:a.H():this.l};
m.ca=function(a){var b;K("use_shared_nsm")&&this.j?null===(b=this.j)||void 0===b?void 0:b.j=a:a!==this.l&&(this.l=a)};
m.ac=function(a){!K("use_shared_nsm")&&(this.m=!0,void 0===a?0:a)&&(this.M||Nm(this))};
m.ab=function(){var a=window.navigator.onLine;return void 0===a?!0:a};
m.Sb=function(){this.S=!0};
m.W=function(a,b){return K("use_shared_nsm")&&this.j?this.j.W(a,b):Be.prototype.W.call(this,a,b)};
function Lm(a){window.addEventListener("online",function(){return w(function(b){if(1==b.h)return v(b,a.fa(),2);a.S&&Im();b.h=0})})}
function Km(a){window.addEventListener("offline",function(){return w(function(b){return v(b,a.fa(),0)})})}
function Nm(a){a.M=Ph(function(){return w(function(b){if(1==b.h)return a.l?a.ab()||!a.m?b.s(3):v(b,a.fa(),3):v(b,a.fa(),3);Nm(a);b.h=0})},Jm)}
m.fa=function(a){var b=this;if(K("use_shared_nsm")&&this.j){var c=Fe(this.j,a);c.then(function(d){K("use_cfr_monitor")&&Em().requestComplete("generate_204",d)});
return c}return this.u?this.u:this.u=new Promise(function(d){var e,f,g;return w(function(h){switch(h.h){case 1:return e=window.AbortController?new window.AbortController:void 0,f=null===e||void 0===e?void 0:e.signal,g=!1,ta(h,2,3),e&&(b.A=Vh.N(function(){e.abort()},a||2E4)),v(h,fetch("/generate_204",{method:"HEAD",
signal:f}),5);case 5:g=!0;case 3:xa(h);K("use_cfr_monitor")&&Em().requestComplete("generate_204",g);b.u=void 0;b.A&&Vh.U(b.A);g!==b.l&&(b.l=g,b.l&&b.m?Ce(b,"ytnetworkstatus-online"):b.m&&Ce(b,"ytnetworkstatus-offline"));d(g);ya(h);break;case 2:wa(h),g=!1,h.s(3)}})})};
O.prototype.sendNetworkCheckRequest=O.prototype.fa;O.prototype.listen=O.prototype.W;O.prototype.enableErrorFlushing=O.prototype.Sb;O.prototype.getWindowStatus=O.prototype.ab;O.prototype.monitorNetworkStatusChange=O.prototype.ac;O.prototype.networkStatusHint=O.prototype.ca;O.prototype.isNetworkAvailable=O.prototype.H;O.getInstance=Mm;function Om(a){a=void 0===a?{}:a;Be.call(this);var b=this;this.l=this.M=0;this.m="ytnetworkstatus-offline";this.u="ytnetworkstatus-online";K("use_shared_nsm")&&(this.m="networkstatus-offline",this.u="networkstatus-online");this.j=Mm();var c=A("yt.networkStatusManager.instance.monitorNetworkStatusChange").bind(this.j);c&&c(a.tb);a.Oa&&!K("use_shared_nsm")&&(c=A("yt.networkStatusManager.instance.enableErrorFlushing").bind(this.j))&&c();if(c=A("yt.networkStatusManager.instance.listen").bind(this.j))a.Ta?
(this.Ta=a.Ta,c(this.u,function(){Pm(b,"publicytnetworkstatus-online");K("use_shared_nsm")&&a.Oa&&Im()}),c(this.m,function(){Pm(b,"publicytnetworkstatus-offline")})):(c(this.u,function(){Ce(b,"publicytnetworkstatus-online");
K("use_shared_nsm")&&a.Oa&&Im()}),c(this.m,function(){Ce(b,"publicytnetworkstatus-offline")}))}
r(Om,Be);Om.prototype.H=function(){var a=A("yt.networkStatusManager.instance.isNetworkAvailable");return a?a.bind(this.j)():!0};
Om.prototype.ca=function(a){var b=A("yt.networkStatusManager.instance.networkStatusHint").bind(this.j);b&&b(a)};
Om.prototype.fa=function(a){var b=this,c;return w(function(d){c=A("yt.networkStatusManager.instance.sendNetworkCheckRequest").bind(b.j);return K("skip_network_check_if_cfr")&&Em().isEndpointCFR("generate_204")?d.return(new Promise(function(e){var f;b.ca((null===(f=window.navigator)||void 0===f?void 0:f.onLine)||!0);e(b.H())})):c?d.return(c(a)):d.return(!0)})};
function Pm(a,b){a.Ta?a.l?(Vh.U(a.M),a.M=Vh.N(function(){a.A!==b&&(Ce(a,b),a.A=b,a.l=N())},a.Ta-(N()-a.l))):(Ce(a,b),a.A=b,a.l=N()):Ce(a,b)}
;var Qm;function Rm(){Ml.call(this,{K:{Pb:sm,qa:rm,wb:om,Zb:pm,jb:qm,set:mm},J:Sm(),handleError:M,ra:Bh,ea:Tm,now:N,Kb:Hm,V:Uh(),ib:"publicytnetworkstatus-online",hb:"publicytnetworkstatus-offline",La:!0,Ka:.1,Ra:rh("potential_esf_error_limit",10),C:K,xa:!(tk()&&Um())});this.j=new jd;K("networkless_immediately_drop_all_requests")&&tm();Ll("LogsDatabaseV2")}
r(Rm,Ml);function Vm(){var a=A("yt.networklessRequestController.instance");a||(a=new Rm,y("yt.networklessRequestController.instance",a,void 0),K("networkless_logging")&&Bl().then(function(b){a.v=b;Nl(a);a.j.resolve();a.La&&Math.random()<=a.Ka&&a.v&&xm(a.v);K("networkless_immediately_drop_sw_health_store")&&Wm(a)}));
return a}
Rm.prototype.writeThenSend=function(a,b){b||(b={});tk()||(this.h=!1);Ml.prototype.writeThenSend.call(this,a,b)};
Rm.prototype.sendThenWrite=function(a,b,c){b||(b={});tk()||(this.h=!1);Ml.prototype.sendThenWrite.call(this,a,b,c)};
Rm.prototype.sendAndWrite=function(a,b){b||(b={});tk()||(this.h=!1);Ml.prototype.sendAndWrite.call(this,a,b)};
Rm.prototype.awaitInitialization=function(){return this.j.promise};
function Wm(a){var b;w(function(c){if(!a.v)throw b=Ik("clearSWHealthLogsDb"),b;return c.return(ym(a.v).catch(function(d){a.handleError(d)}))})}
function Tm(a,b,c){K("use_cfr_monitor")&&Xm(a,b);var d;if(null===(d=b.postParams)||void 0===d?0:d.requestTimeMs)b.postParams.requestTimeMs=Math.round(N());c&&0===Object.keys(b).length?Bm(a):xi(a,b)}
function Sm(){Qm||(Qm=new Om({Oa:!0,tb:!0}));return Qm}
function Xm(a,b){var c=b.onError?b.onError:function(){};
b.onError=function(e,f){Em().requestComplete(a,!1);c(e,f)};
var d=b.onSuccess?b.onSuccess:function(){};
b.onSuccess=function(e,f){Em().requestComplete(a,!0);d(e,f)}}
function Um(){return K("embeds_web_nwl_disable_nocookie")?"www.youtube-nocookie.com"!==Yb(document.location.toString()):!0}
;var Ym=!1,Zm=0,$m=0,an,bn=x.ytNetworklessLoggingInitializationOptions||{isNwlInitialized:Ym,potentialEsfErrorCounter:$m};y("ytNetworklessLoggingInitializationOptions",bn,void 0);
function cn(){var a;w(function(b){switch(b.h){case 1:return v(b,Bl(),2);case 2:a=b.i;if(!a||!tk()&&!K("nwl_init_require_datasync_id_killswitch")||!Um()){b.s(0);break}Ym=!0;bn.isNwlInitialized=Ym;if(!K("use_new_nwl_initialization")){b.s(4);break}return v(b,Vm().awaitInitialization(),5);case 5:return dn().W("publicytnetworkstatus-online",en),dn().W("publicytnetworkstatus-offline",fn),b.return();case 4:return v(b,Ll("LogsDatabaseV2"),6);case 6:if(!(.1>=Math.random())){b.s(7);break}return v(b,sm(a),8);
case 8:return v(b,xm(a),7);case 7:gn();dn().H()&&en();dn().W("publicytnetworkstatus-online",en);dn().W("publicytnetworkstatus-offline",fn);if(!K("networkless_immediately_drop_sw_health_store")){b.s(10);break}return v(b,hn(),10);case 10:if(K("networkless_immediately_drop_all_requests"))return v(b,tm(),0);b.s(0)}})}
function jn(a,b){function c(d){var e=dn().H();if(!kn()||!d||e&&K("vss_networkless_bypass_write"))ln(a,b);else{var f={url:a,options:b,timestamp:N(),status:"NEW",sendCount:0};mm(f,d).then(function(g){f.id=g;dn().H()&&mn(f)}).catch(function(g){mn(f);
dn().H()?M(g):Hm(g)})}}
b=void 0===b?{}:b;K("skip_is_supported_killswitch")?Bl().then(function(d){c(d)}):c(Al())}
function nn(a,b){function c(d){if(kn()&&d){var e={url:a,options:b,timestamp:N(),status:"NEW",sendCount:0},f=!1,g=b.onSuccess?b.onSuccess:function(){};
e.options.onSuccess=function(k,l){K("use_cfr_monitor")&&Em().requestComplete(e.url,!0);void 0!==e.id?rm(e.id,d):f=!0;K("vss_network_hint")&&dn().ca(!0);g(k,l)};
if(K("use_cfr_monitor")){var h=b.onError?b.onError:function(){};
e.options.onError=function(k,l){Em().requestComplete(e.url,!1);h(k,l)}}ln(e.url,e.options);
mm(e,d).then(function(k){e.id=k;f&&rm(e.id,d)}).catch(function(k){dn().H()?M(k):Hm(k)})}else ln(a,b)}
b=void 0===b?{}:b;K("skip_is_supported_killswitch")?Bl().then(function(d){c(d)}):c(Al())}
function en(){var a=Al();if(!a)throw Ik("throttleSend");Zm||(Zm=Vh.N(function(){var b;return w(function(c){if(1==c.h)return v(c,om("NEW",a),2);if(3!=c.h)return b=c.i,b?v(c,mn(b),3):(fn(),c.return());Zm&&(Zm=0,en());c.h=0})},100))}
function fn(){Vh.U(Zm);Zm=0}
function mn(a){var b,c,d;return w(function(e){switch(e.h){case 1:b=Al();if(!b)throw c=Ik("immediateSend"),c;if(void 0===a.id){e.s(2);break}return v(e,pm(a.id,b),3);case 3:(d=e.i)?a=d:Bh(Error("The request cannot be found in the database."));case 2:if(on(a,2592E6)){e.s(4);break}Bh(Error("Networkless Logging: Stored logs request expired age limit"));if(void 0===a.id){e.s(5);break}return v(e,rm(a.id,b),5);case 5:return e.return();case 4:a.skipRetry||(a=pn(a));var f=a,g,h;if(null===(h=null===(g=null===
f||void 0===f?void 0:f.options)||void 0===g?void 0:g.postParams)||void 0===h?0:h.requestTimeMs)f.options.postParams.requestTimeMs=Math.round(N());a=f;if(!a){e.s(0);break}if(!a.skipRetry||void 0===a.id){e.s(8);break}return v(e,rm(a.id,b),8);case 8:ln(a.url,a.options,!!a.skipRetry),e.h=0}})}
function pn(a){var b=Al();if(!b)throw Ik("updateRequestHandlers");var c=a.options.onError?a.options.onError:function(){};
a.options.onError=function(e,f){var g;return w(function(h){switch(h.h){case 1:K("use_cfr_monitor")&&Em().requestComplete(a.url,!1);g=Tl(f);if(!(K("nwl_consider_error_code")&&g||!K("nwl_consider_error_code")&&qn()<=rh("potential_esf_error_limit",10))){h.s(2);break}if(K("skip_checking_network_on_cfr_failure")&&(!K("skip_checking_network_on_cfr_failure")||Em().isEndpointCFR(a.url))){h.s(3);break}return v(h,dn().fa(),3);case 3:if(dn().H()){h.s(2);break}c(e,f);if(!K("nwl_consider_error_code")||void 0===
(null===a||void 0===a?void 0:a.id)){h.s(6);break}return v(h,qm(a.id,b,!1),6);case 6:return h.return();case 2:if(K("nwl_consider_error_code")&&!g&&qn()>rh("potential_esf_error_limit",10))return h.return();A("ytNetworklessLoggingInitializationOptions")&&bn.potentialEsfErrorCounter++;$m++;if(void 0===(null===a||void 0===a?void 0:a.id)){h.s(8);break}return 1>a.sendCount?v(h,qm(a.id,b),12):v(h,rm(a.id,b),8);case 12:Vh.N(function(){dn().H()&&en()},5E3);
case 8:c(e,f),h.h=0}})};
var d=a.options.onSuccess?a.options.onSuccess:function(){};
a.options.onSuccess=function(e,f){return w(function(g){if(1==g.h)return K("use_cfr_monitor")&&Em().requestComplete(a.url,!0),void 0===(null===a||void 0===a?void 0:a.id)?g.s(2):v(g,rm(a.id,b),2);K("vss_network_hint")&&dn().ca(!0);d(e,f);g.h=0})};
return a}
function on(a,b){a=a.timestamp;return N()-a>=b?!1:!0}
function gn(){var a=Al();if(!a)throw Ik("retryQueuedRequests");om("QUEUED",a).then(function(b){b&&!on(b,12E4)?Vh.N(function(){return w(function(c){if(1==c.h)return void 0===b.id?c.s(2):v(c,qm(b.id,a),2);gn();c.h=0})}):dn().H()&&en()})}
function hn(){var a,b;return w(function(c){a=Al();if(!a)throw b=Ik("clearSWHealthLogsDb"),b;return c.return(ym(a).catch(function(d){M(d)}))})}
function dn(){if(K("use_new_nwl"))return Sm();an||(an=new Om({Oa:!0,tb:!0}));return an}
function ln(a,b,c){c&&0===Object.keys(b).length?Bm(a):xi(a,b)}
function kn(){return A("ytNetworklessLoggingInitializationOptions")?bn.isNwlInitialized:Ym}
function qn(){return A("ytNetworklessLoggingInitializationOptions")?bn.potentialEsfErrorCounter:$m}
;function rn(a){var b=this;this.config_=null;a?this.config_=a:Rj()&&(this.config_=lj());Ph(function(){ek(b)},5E3)}
rn.prototype.isReady=function(){!this.config_&&Rj()&&(this.config_=lj());return!!this.config_};
function pj(a,b,c,d){function e(D){D=void 0===D?!1:D;var E;if(d.retry&&"www.youtube-nocookie.com"!=h&&(D||K("skip_ls_gel_retry")||"application/json"!==g.headers["Content-Type"]||(E=ck(b,c,l,k)),E)){var L=g.onSuccess,P=g.onFetchSuccess;g.onSuccess=function(S,W){dk(E);L(S,W)};
c.onFetchSuccess=function(S,W){dk(E);P(S,W)}}try{D&&d.retry&&!d.Bb.bypassNetworkless?(g.method="POST",d.Bb.writeThenSend?K("use_new_nwl")?Vm().writeThenSend(u,g):jn(u,g):K("use_new_nwl")?Vm().sendAndWrite(u,g):nn(u,g)):(g.method="POST",g.postParams||(g.postParams={}),xi(u,g))}catch(S){if("InvalidAccessError"==S.name)E&&(dk(E),E=0),Bh(Error("An extension is blocking network request."));
else throw S;}E&&Ph(function(){ek(a)},5E3)}
!B("VISITOR_DATA")&&"visitor_id"!==b&&.01>Math.random()&&Bh(new sk("Missing VISITOR_DATA when sending innertube request.",b,c,d));if(!a.isReady()){var f=new sk("innertube xhrclient not ready",b,c,d);M(f);throw f;}var g={headers:d.headers||{},method:"POST",postParams:c,postBody:d.postBody,postBodyFormat:d.postBodyFormat||"JSON",onTimeout:function(){d.onTimeout()},
onFetchTimeout:d.onTimeout,onSuccess:function(D,E){if(d.onSuccess)d.onSuccess(E)},
onFetchSuccess:function(D){if(d.onSuccess)d.onSuccess(D)},
onError:function(D,E){if(d.onError)d.onError(E)},
onFetchError:function(D){if(d.onError)d.onError(D)},
timeout:d.timeout,withCredentials:!0};g.headers["Content-Type"]||(g.headers["Content-Type"]="application/json");var h="";(f=a.config_.Wb)&&(h=f);var k=a.config_.Yb||!1,l=Xj(k,h,d);Object.assign(g.headers,l);(f=g.headers.Authorization)&&!h&&(g.headers["x-origin"]=window.location.origin);var n="/youtubei/"+a.config_.innertubeApiVersion+"/"+b,t={alt:"json"},z=a.config_.Xb&&f;z=z&&f.startsWith("Bearer");z||(t.key=a.config_.innertubeApiKey);var u=li(""+h+n,t||{},!0);K("use_new_nwl")&&Vm().h||!K("use_new_nwl")&&
kn()?zl().then(function(D){e(D)}):e(!1)}
;function pk(a,b,c){c=void 0===c?{}:c;var d=rn;B("ytLoggingEventsDefaultDisabled",!1)&&rn==rn&&(d=null);wj(a,b,d,c)}
;var sn=[{gb:function(a){return"Cannot read property '"+a.key+"'"},
Qa:{Error:[{regexp:/(Permission denied) to access property "([^']+)"/,groups:["reason","key"]}],TypeError:[{regexp:/Cannot read property '([^']+)' of (null|undefined)/,groups:["key","value"]},{regexp:/\u65e0\u6cd5\u83b7\u53d6\u672a\u5b9a\u4e49\u6216 (null|undefined) \u5f15\u7528\u7684\u5c5e\u6027\u201c([^\u201d]+)\u201d/,groups:["value","key"]},{regexp:/\uc815\uc758\ub418\uc9c0 \uc54a\uc74c \ub610\ub294 (null|undefined) \ucc38\uc870\uc778 '([^']+)' \uc18d\uc131\uc744 \uac00\uc838\uc62c \uc218 \uc5c6\uc2b5\ub2c8\ub2e4./,
groups:["value","key"]},{regexp:/No se puede obtener la propiedad '([^']+)' de referencia nula o sin definir/,groups:["key"]},{regexp:/Unable to get property '([^']+)' of (undefined or null) reference/,groups:["key","value"]},{regexp:/(null) is not an object \(evaluating '(?:([^.]+)\.)?([^']+)'\)/,groups:["value","base","key"]}]}},{gb:function(a){return"Cannot call '"+a.key+"'"},
Qa:{TypeError:[{regexp:/(?:([^ ]+)?\.)?([^ ]+) is not a function/,groups:["base","key"]},{regexp:/([^ ]+) called on (null or undefined)/,groups:["key","value"]},{regexp:/Object (.*) has no method '([^ ]+)'/,groups:["base","key"]},{regexp:/Object doesn't support property or method '([^ ]+)'/,groups:["key"]},{regexp:/\u30aa\u30d6\u30b8\u30a7\u30af\u30c8\u306f '([^']+)' \u30d7\u30ed\u30d1\u30c6\u30a3\u307e\u305f\u306f\u30e1\u30bd\u30c3\u30c9\u3092\u30b5\u30dd\u30fc\u30c8\u3057\u3066\u3044\u307e\u305b\u3093/,
groups:["key"]},{regexp:/\uac1c\uccb4\uac00 '([^']+)' \uc18d\uc131\uc774\ub098 \uba54\uc11c\ub4dc\ub97c \uc9c0\uc6d0\ud558\uc9c0 \uc54a\uc2b5\ub2c8\ub2e4./,groups:["key"]}]}},{gb:function(a){return a.key+" is not defined"},
Qa:{ReferenceError:[{regexp:/(.*) is not defined/,groups:["key"]},{regexp:/Can't find variable: (.*)/,groups:["key"]}]}}];var un={ia:[],ha:[{la:tn,weight:500}]};function tn(a){if("JavaException"===a.name)return!0;a=a.stack;return a.includes("chrome://")||a.includes("chrome-extension://")||a.includes("moz-extension://")}
;function vn(){this.ha=[];this.ia=[]}
var wn;function xn(){if(!wn){var a=wn=new vn;a.ia.length=0;a.ha.length=0;un.ia&&a.ia.push.apply(a.ia,un.ia);un.ha&&a.ha.push.apply(a.ha,un.ha)}return wn}
;var yn=new J;function zn(a){function b(){return a.charCodeAt(d++)}
var c=a.length,d=0;do{var e=An(b);if(Infinity===e)break;var f=e>>3;switch(e&7){case 0:e=An(b);if(2===f)return e;break;case 1:if(2===f)return;d+=8;break;case 2:e=An(b);if(2===f)return a.substr(d,e);d+=e;break;case 5:if(2===f)return;d+=4;break;default:return}}while(d<c)}
function An(a){var b=a(),c=b&127;if(128>b)return c;b=a();c|=(b&127)<<7;if(128>b)return c;b=a();c|=(b&127)<<14;if(128>b)return c;b=a();return 128>b?c|(b&127)<<21:Infinity}
;function Bn(a,b,c,d){if(a)if(Array.isArray(a)){var e=d;for(d=0;d<a.length&&!(a[d]&&(e+=Cn(d,a[d],b,c),500<e));d++);d=e}else if("object"===typeof a)for(e in a){if(a[e]){var f=a[e];var g=b;var h=c;g="string"!==typeof f||"clickTrackingParams"!==e&&"trackingParams"!==e?0:(f=zn(atob(f.replace(/-/g,"+").replace(/_/g,"/"))))?Cn(e+".ve",f,g,h):0;d+=g;d+=Cn(e,a[e],b,c);if(500<d)break}}else c[b]=Dn(a),d+=c[b].length;else c[b]=Dn(a),d+=c[b].length;return d}
function Cn(a,b,c,d){c+="."+a;a=Dn(b);d[c]=a;return c.length+a.length}
function Dn(a){try{return("string"===typeof a?a:String(JSON.stringify(a))).substr(0,500)}catch(b){return"unable to serialize "+typeof a+" ("+b.message+")"}}
;var En=new Set,Fn=0,Gn=0,Hn=0,In=[],Jn=["PhantomJS","Googlebot","TO STOP THIS SECURITY SCAN go/scan"];function ok(a){Kn(a)}
function Ln(a){Kn(a,"WARNING")}
function Kn(a,b,c,d,e,f){f=void 0===f?{}:f;f.name=c||B("INNERTUBE_CONTEXT_CLIENT_NAME",1);f.version=d||B("INNERTUBE_CONTEXT_CLIENT_VERSION",void 0);c=f||{};b=void 0===b?"ERROR":b;b=void 0===b?"ERROR":b;if(a&&(a.hasOwnProperty("level")&&a.level&&(b=a.level),K("console_log_js_exceptions")&&(d=[],d.push("Name: "+a.name),d.push("Message: "+a.message),a.hasOwnProperty("params")&&d.push("Error Params: "+JSON.stringify(a.params)),a.hasOwnProperty("args")&&d.push("Error args: "+JSON.stringify(a.args)),d.push("File name: "+
a.fileName),d.push("Stacktrace: "+a.stack),window.console.log(d.join("\n"),a)),!(5<=Fn))){d=In;var g=$d(a);e=g.message||"Unknown Error";f=g.name||"UnknownError";var h=g.stack||a.i||"Not available";if(h.startsWith(f+": "+e)){var k=h.split("\n");k.shift();h=k.join("\n")}k=g.lineNumber||"Not available";g=g.fileName||"Not available";var l=0;if(a.hasOwnProperty("args")&&a.args&&a.args.length)for(var n=0;n<a.args.length&&!(l=Bn(a.args[n],"params."+n,c,l),500<=l);n++);else if(a.hasOwnProperty("params")&&
a.params){var t=a.params;if("object"===typeof a.params)for(n in t){if(t[n]){var z="params."+n,u=Dn(t[n]);c[z]=u;l+=z.length+u.length;if(500<l)break}}else c.params=Dn(t)}if(d.length)for(n=0;n<d.length&&!(l=Bn(d[n],"params.context."+n,c,l),500<=l);n++);navigator.vendor&&!c.hasOwnProperty("vendor")&&(c["device.vendor"]=navigator.vendor);n={message:e,name:f,lineNumber:k,fileName:g,stack:h,params:c,sampleWeight:1};c=Number(a.columnNumber);isNaN(c)||(n.lineNumber=n.lineNumber+":"+c);if("IGNORED"===a.level)a=
0;else a:{a=xn();c=q(a.ia);for(d=c.next();!d.done;d=c.next())if(d=d.value,n.message&&n.message.match(d.Ao)){a=d.weight;break a}a=q(a.ha);for(c=a.next();!c.done;c=a.next())if(c=c.value,c.la(n)){a=c.weight;break a}a=1}n.sampleWeight=a;a=q(sn);for(c=a.next();!c.done;c=a.next())if(c=c.value,c.Qa[n.name])for(e=q(c.Qa[n.name]),d=e.next();!d.done;d=e.next())if(f=d.value,d=n.message.match(f.regexp)){n.params["params.error.original"]=d[0];e=f.groups;f={};for(k=0;k<e.length;k++)f[e[k]]=d[k+1],n.params["params.error."+
e[k]]=d[k+1];n.message=c.gb(f);break}n.params||(n.params={});a=xn();n.params["params.errorServiceSignature"]="msg="+a.ia.length+"&cb="+a.ha.length;n.params["params.serviceWorker"]="false";x.document&&x.document.querySelectorAll&&(n.params["params.fscripts"]=String(document.querySelectorAll("script:not([nonce])").length));Ab("sample").constructor!==yb&&(n.params["params.fconst"]="true");window.yterr&&"function"===typeof window.yterr&&window.yterr(n);if(0!==n.sampleWeight&&!En.has(n.message)){"ERROR"===
b?(yn.ka("handleError",n),K("record_app_crashed_web")&&0===Hn&&1===n.sampleWeight&&(Hn++,a={appCrashType:"APP_CRASH_TYPE_BREAKPAD"},K("report_client_error_with_app_crash_ks")||(a.systemHealth={crashData:{clientError:{logMessage:{message:n.message}}}}),pk("appCrashed",a)),Gn++):"WARNING"===b&&yn.ka("handleWarning",n);if(K("kevlar_gel_error_routing")){a=b;b:{c=q(Jn);for(d=c.next();!d.done;d=c.next())if(gk(d.value.toLowerCase())){c=!0;break b}c=!1}if(c)c=void 0;else{d={stackTrace:n.stack};n.fileName&&
(d.filename=n.fileName);c=n.lineNumber&&n.lineNumber.split?n.lineNumber.split(":"):[];0!==c.length&&(1!==c.length||isNaN(Number(c[0]))?2!==c.length||isNaN(Number(c[0]))||isNaN(Number(c[1]))||(d.lineNumber=Number(c[0]),d.columnNumber=Number(c[1])):d.lineNumber=Number(c[0]));c={level:"ERROR_LEVEL_UNKNOWN",message:n.message,errorClassName:n.name,sampleWeight:n.sampleWeight};"ERROR"===a?c.level="ERROR_LEVEL_ERROR":"WARNING"===a&&(c.level="ERROR_LEVEL_WARNNING");d={isObfuscated:!0,browserStackInfo:d};
e={pageUrl:window.location.href,kvPairs:[]};B("FEXP_EXPERIMENTS")&&(e.experimentIds=B("FEXP_EXPERIMENTS"));f=B("LATEST_ECATCHER_SERVICE_TRACKING_PARAMS",void 0);k=ih.EXPERIMENT_FLAGS;if((!k||!k.web_disable_gel_stp_ecatcher_killswitch)&&f)for(g=q(Object.keys(f)),k=g.next();!k.done;k=g.next())k=k.value,e.kvPairs.push({key:k,value:String(f[k])});if(f=n.params)for(g=q(Object.keys(f)),k=g.next();!k.done;k=g.next())k=k.value,e.kvPairs.push({key:"client."+k,value:String(f[k])});f=ph("SERVER_NAME");k=ph("SERVER_VERSION");
f&&k&&(e.kvPairs.push({key:"server.name",value:f}),e.kvPairs.push({key:"server.version",value:k}));c={errorMetadata:e,stackTrace:d,logMessage:c}}c&&(pk("clientError",c),("ERROR"===a||K("errors_flush_gel_always_killswitch"))&&ej())}if(!K("suppress_error_204_logging")){a=n.params||{};b={urlParams:{a:"logerror",t:"jserror",type:n.name,msg:n.message.substr(0,250),line:n.lineNumber,level:b,"client.name":a.name},postParams:{url:B("PAGE_NAME",window.location.href),file:n.fileName},method:"POST"};a.version&&
(b["client.version"]=a.version);if(b.postParams){n.stack&&(b.postParams.stack=n.stack);c=q(Object.keys(a));for(d=c.next();!d.done;d=c.next())d=d.value,b.postParams["client."+d]=a[d];if(a=B("LATEST_ECATCHER_SERVICE_TRACKING_PARAMS",void 0))for(c=q(Object.keys(a)),d=c.next();!d.done;d=c.next())d=d.value,b.postParams[d]=a[d];a=B("SERVER_NAME",void 0);c=B("SERVER_VERSION",void 0);a&&c&&(b.postParams["server.name"]=a,b.postParams["server.version"]=c)}xi(B("ECATCHER_REPORT_HOST","")+"/error_204",b)}try{En.add(n.message)}catch(D){}Fn++}}}
function Mn(a){var b=Fa.apply(1,arguments),c=a;c.args||(c.args=[]);c.args.push.apply(c.args,ha(b))}
;function Nn(){this.register=new Map}
function On(a){a=q(a.register.values());for(var b=a.next();!b.done;b=a.next())b.value.Do("ABORTED")}
Nn.prototype.clear=function(){On(this);this.register.clear()};
var Pn=new Nn;var Qn=Date.now().toString();
function Rn(){a:{if(window.crypto&&window.crypto.getRandomValues)try{var a=Array(16),b=new Uint8Array(16);window.crypto.getRandomValues(b);for(var c=0;c<a.length;c++)a[c]=b[c];var d=a;break a}catch(e){}d=Array(16);for(a=0;16>a;a++){b=Date.now();for(c=0;c<b%23;c++)d[a]=Math.random();d[a]=Math.floor(256*Math.random())}if(Qn)for(a=1,b=0;b<Qn.length;b++)d[a%16]=d[a%16]^d[(a-1)%16]/4^Qn.charCodeAt(b),a++}a=[];for(b=0;b<d.length;b++)a.push("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_".charAt(d[b]&63));
return a.join("")}
;var Sn,Tn=x.ytLoggingDocDocumentNonce_;Tn||(Tn=Rn(),Wa("ytLoggingDocDocumentNonce_",Tn));Sn=Tn;var Un={rg:0,md:1,ud:2,Tj:3,tg:4,Qn:5,Jk:6,jm:7,Ll:8,0:"DEFAULT",1:"CHAT",2:"CONVERSATIONS",3:"MINIPLAYER",4:"DIALOG",5:"VOZ",6:"MUSIC_WATCH_TABS",7:"SHARE",8:"PUSH_NOTIFICATIONS"};function Vn(a){this.h=a}
function Wn(a){return new Vn({trackingParams:a})}
Vn.prototype.getAsJson=function(){var a={};void 0!==this.h.trackingParams?a.trackingParams=this.h.trackingParams:(a.veType=this.h.veType,void 0!==this.h.veCounter&&(a.veCounter=this.h.veCounter),void 0!==this.h.elementIndex&&(a.elementIndex=this.h.elementIndex));void 0!==this.h.dataElement&&(a.dataElement=this.h.dataElement.getAsJson());void 0!==this.h.youtubeData&&(a.youtubeData=this.h.youtubeData);return a};
Vn.prototype.getAsJspb=function(){var a=new zg;void 0!==this.h.trackingParams?F(a,1,this.h.trackingParams):(void 0!==this.h.veType&&F(a,2,this.h.veType),void 0!==this.h.veCounter&&F(a,6,this.h.veCounter),void 0!==this.h.elementIndex&&F(a,3,this.h.elementIndex));if(void 0!==this.h.dataElement){var b=this.h.dataElement.getAsJspb();G(a,7,b)}void 0!==this.h.youtubeData&&G(a,8,this.h.jspbYoutubeData);return a};
Vn.prototype.toString=function(){return JSON.stringify(this.getAsJson())};
Vn.prototype.isClientVe=function(){return!this.h.trackingParams&&!!this.h.veType};function Xn(a){a=void 0===a?0:a;return 0==a?"client-screen-nonce":"client-screen-nonce."+a}
function Yn(a){a=void 0===a?0:a;return 0==a?"ROOT_VE_TYPE":"ROOT_VE_TYPE."+a}
function Zn(a){return B(Yn(void 0===a?0:a),void 0)}
y("yt_logging_screen.getRootVeType",Zn,void 0);function $n(a){return(a=Zn(void 0===a?0:a))?new Vn({veType:a,youtubeData:void 0,jspbYoutubeData:void 0}):null}
function ao(){var a=B("csn-to-ctt-auth-info");a||(a={},oh("csn-to-ctt-auth-info",a));return a}
function bo(a){a=void 0===a?0:a;var b=B(Xn(a));if(!b&&!B("USE_CSN_FALLBACK",!0))return null;b||!K("use_undefined_csn_any_layer")&&0!=a||(b="UNDEFINED_CSN");return b?b:null}
y("yt_logging_screen.getCurrentCsn",bo,void 0);function co(a,b,c){var d=ao();(c=bo(c))&&delete d[c];b&&(d[a]=b)}
function eo(a){return ao()[a]}
y("yt_logging_screen.getCttAuthInfo",eo,void 0);function fo(a,b,c,d){c=void 0===c?0:c;if(a!==B(Xn(c))||b!==B(Yn(c)))co(a,d,c),oh(Xn(c),a),oh(Yn(c),b),b=function(){setTimeout(function(){if(a){var e={clientDocumentNonce:Sn,clientScreenNonce:a};K("use_default_heartbeat_client")?pk("foregroundHeartbeatScreenAssociated",e):wj("foregroundHeartbeatScreenAssociated",e,rn)}},0)},"requestAnimationFrame"in window?window.requestAnimationFrame(b):b()}
y("yt_logging_screen.setCurrentScreen",fo,void 0);var go=window.yt&&window.yt.msgs_||window.ytcfg&&window.ytcfg.msgs||{};y("yt.msgs_",go,void 0);function ho(a){dh(go,arguments)}
;var io={ld:3611,Ac:27686,Bc:85013,Cc:23462,Ec:42016,Fc:62407,Gc:26926,Dc:43781,Hc:51236,Ic:79148,Jc:50160,Kc:77504,Wc:87907,Xc:18630,Yc:54445,Zc:80935,bd:105675,cd:37521,dd:47786,ed:98349,fd:123695,gd:6827,hd:29434,jd:7282,kd:124448,od:32276,nd:76278,pd:93911,qd:106531,rd:27259,sd:27262,td:27263,vd:21759,wd:27107,xd:62936,yd:49568,zd:38408,Ad:80637,Bd:68727,Cd:68728,Dd:80353,Ed:80356,Fd:74610,Gd:45707,Hd:83962,Id:83970,Jd:46713,Kd:89711,Ld:74612,Md:93265,Nd:74611,Od:131380,Qd:128979,Rd:139311,Sd:128978,
Pd:131391,Td:105350,Vd:139312,Wd:134800,Ud:131392,Yd:113533,Zd:93252,ae:99357,ce:94521,de:114252,ee:113532,ge:94522,be:94583,he:88E3,ie:139580,je:93253,ke:93254,le:94387,me:94388,ne:93255,oe:97424,Xd:72502,pe:110111,qe:76019,se:117092,te:117093,re:89431,ue:110466,we:77240,xe:60508,ye:137401,ze:137402,Ae:137046,Be:73393,Ce:113534,De:92098,Ee:131381,Fe:84517,Ge:83759,He:80357,Ie:86113,Je:72598,Ke:72733,Le:107349,Me:124275,Ne:118203,Oe:133275,Pe:133274,Qe:133272,Re:133273,Se:133276,Te:144507,Ue:143247,
Ve:143248,We:143249,Xe:143250,Ye:143251,Ze:144401,bf:117431,af:133797,cf:128572,df:133405,ef:117429,ff:117430,gf:117432,hf:120080,jf:117259,kf:121692,lf:145656,mf:145655,nf:145653,pf:145654,qf:145657,rf:132972,sf:133051,tf:133658,uf:132971,vf:97615,xf:143359,wf:143356,zf:143361,yf:143358,Bf:143360,Af:143357,Cf:142303,Df:143353,Ef:143354,Ff:144479,Gf:143355,Hf:31402,Jf:133624,Kf:133623,Lf:133622,If:133621,Mf:84774,Nf:95117,Of:98930,Pf:98931,Qf:98932,Rf:43347,Sf:129889,Tf:45474,Uf:100352,Vf:84758,Wf:98443,
Xf:117985,Yf:74613,Zf:74614,ag:64502,cg:136032,dg:74615,eg:74616,fg:122224,gg:74617,hg:77820,ig:74618,jg:93278,kg:93274,lg:93275,mg:93276,ng:22110,og:29433,pg:133798,qg:132295,sg:120541,ug:82047,vg:113550,wg:75836,xg:75837,yg:42352,zg:84512,Ag:76065,Bg:75989,Cg:16623,Dg:32594,Eg:27240,Fg:32633,Gg:74858,Ig:3945,Hg:16989,Jg:45520,Kg:25488,Lg:25492,Mg:25494,Ng:55760,Og:14057,Pg:18451,Qg:57204,Rg:57203,Sg:17897,Tg:57205,Ug:18198,Vg:17898,Wg:17909,Xg:43980,Yg:46220,Zg:11721,ah:49954,bh:96369,dh:3854,eh:56251,
fh:25624,wh:16906,xh:99999,yh:68172,zh:27068,Ah:47973,Bh:72773,Ch:26970,Dh:26971,Eh:96805,Fh:17752,Gh:73233,Hh:109512,Ih:22256,Jh:14115,Kh:22696,Lh:89278,Mh:89277,Nh:109513,Oh:43278,Ph:43459,Qh:43464,Rh:89279,Sh:43717,Th:55764,Uh:22255,Vh:89281,Wh:40963,Xh:43277,Yh:43442,Zh:91824,ai:120137,bi:96367,ci:36850,di:72694,fi:37414,gi:36851,ii:124863,hi:121343,ji:73491,ki:54473,li:43375,mi:46674,ni:143815,oi:139095,ri:144402,si:32473,ti:72901,vi:72906,wi:50947,xi:50612,yi:50613,zi:50942,Ai:84938,Bi:84943,
Ci:84939,Di:84941,Ei:84944,Fi:84940,Gi:84942,Hi:35585,Ii:51926,Ji:79983,Ki:63238,Li:18921,Mi:63241,Ni:57893,Oi:41182,Pi:135732,Qi:33424,Ri:22207,Si:42993,Ti:36229,Ui:22206,Vi:22205,Wi:18993,Xi:19001,Yi:18990,Zi:18991,aj:18997,bj:18725,cj:19003,dj:36874,ej:44763,fj:33427,gj:67793,hj:22182,ij:37091,jj:34650,kj:50617,lj:47261,mj:22287,nj:25144,oj:97917,pj:62397,qj:125598,rj:137935,sj:36961,tj:108035,uj:27426,vj:27857,wj:27846,xj:27854,yj:69692,zj:61411,Aj:39299,Bj:38696,Cj:62520,Dj:36382,Ej:108701,Fj:50663,
Gj:36387,Hj:14908,Ij:37533,Jj:105443,Kj:61635,Lj:62274,Mj:133818,Nj:65702,Oj:65703,Pj:65701,Qj:76256,Rj:37671,Sj:49953,Uj:36216,Vj:28237,Wj:39553,Xj:29222,Yj:26107,Zj:38050,ak:26108,ck:120745,bk:26109,dk:26110,ek:66881,fk:28236,gk:14586,hk:57929,ik:74723,jk:44098,kk:44099,nk:23528,pk:61699,lk:134104,mk:134103,qk:59149,rk:101951,sk:97346,tk:118051,uk:95102,vk:64882,wk:119505,xk:63595,yk:63349,zk:95101,Ak:75240,Bk:27039,Ck:68823,Dk:21537,Ek:83464,Fk:75707,Gk:83113,Hk:101952,Ik:101953,Kk:79610,Lk:125755,
Mk:24402,Nk:24400,Ok:32925,Pk:57173,Qk:122502,Rk:145268,Sk:138480,Tk:64423,Uk:64424,Vk:33986,Wk:100828,Xk:129089,Yk:21409,dl:135155,fl:135156,il:135157,jl:135158,kl:135159,ll:135160,ml:135161,nl:135162,ol:135163,pl:135164,ql:135165,rl:135166,Zk:11070,al:11074,bl:17880,sl:14001,vl:30709,wl:30707,xl:30711,yl:30710,zl:30708,ul:26984,Al:63648,Bl:63649,Cl:51879,Dl:111059,El:5754,Fl:20445,Hl:130975,Gl:130976,Il:110386,Jl:113746,Kl:66557,Ml:17310,Nl:28631,Ol:21589,Pl:68012,Ql:60480,Rl:138664,Sl:141121,Tl:31571,
Ul:141978,Vl:76980,Wl:41577,Xl:45469,Yl:38669,Zl:13768,am:13777,bm:141842,cm:62985,dm:4724,em:59369,fm:43927,gm:43928,hm:12924,im:100355,lm:56219,mm:27669,nm:10337,km:47896,om:122629,qm:139723,pm:139722,rm:121258,sm:107598,tm:127991,um:96639,vm:107536,wm:130169,xm:96661,ym:145188,zm:96658,Am:116646,Bm:121122,Cm:96660,Dm:127738,Em:127083,Fm:104443,Gm:96659,Hm:106442,Im:134840,Jm:63667,Km:63668,Lm:63669,Mm:130686,Nm:78314,Om:55761,Pm:127098,Qm:134841,Rm:96368,Sm:67374,Tm:48992,Um:49956,Vm:31961,Wm:26388,
Xm:23811,Ym:5E4,Zm:126250,an:96370,bn:47355,cn:47356,dn:37935,en:45521,fn:21760,gn:83769,hn:49977,jn:49974,kn:93497,ln:93498,mn:34325,nn:140759,pn:115803,qn:123707,rn:100081,sn:35309,tn:68314,un:25602,vn:100339,wn:143516,xn:59018,yn:18248,zn:50625,An:9729,Bn:37168,Cn:37169,Dn:21667,En:16749,Fn:18635,Gn:39305,Hn:18046,In:53969,Jn:8213,Kn:93926,Ln:102852,Mn:110099,Nn:22678,On:69076,Pn:137575,Rn:139224,Sn:100856,Tn:17736,Un:3832,Vn:55759,Wn:64031,co:93044,eo:93045,fo:34388,ho:17657,jo:17655,ko:39579,
lo:39578,mo:77448,no:8196,oo:11357,po:69877,qo:8197,ro:82039};function jo(){var a=sb(ko),b;return Kf(new Df(function(c,d){a.onSuccess=function(e){ri(e)?c(new lo(e)):d(new mo("Request failed, status="+(e&&"status"in e?e.status:-1),"net.badstatus",e))};
a.onError=function(e){d(new mo("Unknown request error","net.unknown",e))};
a.onTimeout=function(e){d(new mo("Request timed out","net.timeout",e))};
b=xi("//googleads.g.doubleclick.net/pagead/id",a)}),function(c){c instanceof Lf&&b.abort();
return If(c)})}
function mo(a,b,c){$a.call(this,a+", errorCode="+b);this.errorCode=b;this.xhr=c;this.name="PromiseAjaxError"}
r(mo,$a);function lo(a){this.xhr=a}
;function no(){this.i=0;this.h=null}
no.prototype.then=function(a,b,c){return 1===this.i&&a?(a=a.call(c,this.h),Cf(a)?a:oo(a)):2===this.i&&b?(a=b.call(c,this.h),Cf(a)?a:po(a)):this};
no.prototype.getValue=function(){return this.h};
no.prototype.$goog_Thenable=!0;function po(a){var b=new no;a=void 0===a?null:a;b.i=2;b.h=void 0===a?null:a;return b}
function oo(a){var b=new no;a=void 0===a?null:a;b.i=1;b.h=void 0===a?null:a;return b}
;function qo(){if(Td())return!0;var a=B("INNERTUBE_CLIENT_NAME");return!a||"WEB"!==a&&"MWEB"!==a||fk&&gk("applewebkit")&&!gk("version")&&(!gk("safari")||gk("gsa/"))||kc&&gk("version/")?!0:(a=Cj("CONSENT"))?a.startsWith("YES+"):!0}
;function ro(a){$a.call(this,a.message||a.description||a.name);this.isMissing=a instanceof so;this.isTimeout=a instanceof mo&&"net.timeout"==a.errorCode;this.isCanceled=a instanceof Lf}
r(ro,$a);ro.prototype.name="BiscottiError";function so(){$a.call(this,"Biscotti ID is missing from server")}
r(so,$a);so.prototype.name="BiscottiMissingError";var ko={format:"RAW",method:"GET",timeout:5E3,withCredentials:!0},to=null;function bi(){if(K("disable_biscotti_fetch_entirely_for_all_web_clients"))return If(Error("Biscotti id fetching has been disabled entirely."));if(!qo())return If(Error("User has not consented - not fetching biscotti id."));if("1"==qb())return If(Error("Biscotti ID is not available in private embed mode"));to||(to=Kf(jo().then(uo),function(a){return vo(2,a)}));
return to}
function uo(a){a=a.xhr.responseText;if(0!=a.lastIndexOf(")]}'",0))throw new so;a=JSON.parse(a.substr(4));if(1<(a.type||1))throw new so;a=a.id;ci(a);to=oo(a);wo(18E5,2);return a}
function vo(a,b){b=new ro(b);ci("");to=po(b);0<a&&wo(12E4,a-1);throw b;}
function wo(a,b){Lh(function(){Kf(jo().then(uo,function(c){return vo(b,c)}),Ka)},a)}
function xo(){try{var a=A("yt.ads.biscotti.getId_");return a?a():bi()}catch(b){return If(b)}}
;function yo(a){if("1"!=qb()){a&&ai();try{xo().then(function(){},function(){}),Lh(yo,18E5)}catch(b){M(b)}}}
;function zo(){this.xc=!0}
function Ao(a){var b={},c=Vd([]);c&&(b.Authorization=c,c=a=null===a||void 0===a?void 0:a.sessionIndex,void 0===c&&(c=Number(B("SESSION_INDEX",0)),c=isNaN(c)?0:c),b["X-Goog-AuthUser"]=c,"INNERTUBE_HOST_OVERRIDE"in ih||(b["X-Origin"]=window.location.origin),void 0===a&&"DELEGATED_SESSION_ID"in ih&&(b["X-Goog-PageId"]=B("DELEGATED_SESSION_ID")));return b}
;var Bo={identityType:"UNAUTHENTICATED_IDENTITY_TYPE_UNKNOWN"};var Co=new Map([["dark","USER_INTERFACE_THEME_DARK"],["light","USER_INTERFACE_THEME_LIGHT"]]),Do=["/fashion","/feed/fashion_destination","/channel/UCrpQ4p1Ql_hG8rKXIKM1MOQ"];function Eo(){var a=void 0===a?window.location.href:a;if(K("kevlar_disable_theme_param"))return null;var b=Xb(a.match(Wb)[5]||null);if(Fo(b))return"USER_INTERFACE_THEME_DARK";try{var c=ki(a).theme;return Co.get(c)||null}catch(d){}return null}
function Fo(a){var b=Do.map(function(c){return c.toLowerCase()});
return!K("disable_dark_fashion_destination_launch")&&b.some(function(c){return a.toLowerCase().startsWith(c)})?!0:!1}
;function Go(){this.h={};if(this.i=Dj()){var a=Cj("CONSISTENCY");a&&Ho(this,{encryptedTokenJarContents:a})}}
Go.prototype.handleResponse=function(a,b){var c,d,e;b=(null===(d=null===(c=b.ba.context)||void 0===c?void 0:c.request)||void 0===d?void 0:d.consistencyTokenJars)||[];(a=null===(e=a.responseContext)||void 0===e?void 0:e.consistencyTokenJar)&&this.replace(b,a)};
Go.prototype.replace=function(a,b){a=q(a);for(var c=a.next();!c.done;c=a.next())delete this.h[c.value.encryptedTokenJarContents];Ho(this,b)};
function Ho(a,b){if(b.encryptedTokenJarContents&&(a.h[b.encryptedTokenJarContents]=b,"string"===typeof b.expirationSeconds)){var c=Number(b.expirationSeconds);setTimeout(function(){delete a.h[b.encryptedTokenJarContents]},1E3*c);
a.i&&Bj("CONSISTENCY",b.encryptedTokenJarContents,c,void 0,!0)}}
;var Io=window.location.hostname.split(".").slice(-2).join(".");function Jo(){var a=B("LOCATION_PLAYABILITY_TOKEN");"TVHTML5"===B("INNERTUBE_CLIENT_NAME")&&(this.h=Ko(this))&&(a=this.h.get("yt-location-playability-token"));a&&(this.locationPlayabilityToken=a,this.i=void 0)}
var Lo;Jo.getInstance=function(){Lo=A("yt.clientLocationService.instance");Lo||(Lo=new Jo,y("yt.clientLocationService.instance",Lo,void 0));return Lo};
Jo.prototype.setLocationOnInnerTubeContext=function(a){a.client||(a.client={});this.i?(a.client.locationInfo||(a.client.locationInfo={}),a.client.locationInfo.latitudeE7=1E7*this.i.coords.latitude,a.client.locationInfo.longitudeE7=1E7*this.i.coords.longitude,a.client.locationInfo.horizontalAccuracyMeters=this.i.coords.accuracy,a.client.locationInfo.forceLocationPlayabilityTokenRefresh=!0):this.locationPlayabilityToken&&(a.client.locationPlayabilityToken=this.locationPlayabilityToken)};
Jo.prototype.handleResponse=function(a){var b;a=null===(b=a.responseContext)||void 0===b?void 0:b.locationPlayabilityToken;void 0!==a&&(this.locationPlayabilityToken=a,this.i=void 0,"TVHTML5"===B("INNERTUBE_CLIENT_NAME")?(this.h=Ko(this))&&this.h.set("yt-location-playability-token",a,15552E3):Bj("YT_CL",JSON.stringify({loctok:a}),15552E3,Io,!0))};
function Ko(a){return void 0===a.h?new Zj("yt-client-location"):a.h}
Jo.prototype.getCurrentPositionFromGeolocation=function(){var a=this;if(!(navigator&&navigator.geolocation&&navigator.geolocation.getCurrentPosition)||!K("web_enable_browser_geolocation_api")&&!K("enable_handoff_location_2fa_on_mweb"))return Promise.reject(Error("Geolocation unsupported"));var b=!1,c=1E4;K("enable_handoff_location_2fa_on_mweb")&&(b=!0,c=15E3);return new Promise(function(d,e){navigator.geolocation.getCurrentPosition(function(f){a.i=f;d(f)},function(f){e(f)},{enableHighAccuracy:b,
maximumAge:0,timeout:c})})};
Jo.prototype.createUnpluggedLocationInfo=function(a){var b={};a=a.coords;if(null===a||void 0===a?0:a.latitude)b.latitudeE7=Math.floor(1E7*a.latitude);if(null===a||void 0===a?0:a.longitude)b.longitudeE7=Math.floor(1E7*a.longitude);if(null===a||void 0===a?0:a.accuracy)b.locationRadiusMeters=Math.round(a.accuracy);return b};function Mo(a,b){var c,d;if((null===(c=a.signalServiceEndpoint)||void 0===c?0:c.signal)&&b.Aa){var e=b.Aa[a.signalServiceEndpoint.signal];if(e)return e()}if((null===(d=a.continuationCommand)||void 0===d?0:d.request)&&b.Qb&&(e=b.Qb[a.continuationCommand.request]))return e();for(var f in a)if(b.pb[f]&&(a=b.pb[f]))return a()}
;var No=Symbol("injectionDeps");function Oo(){this.name="NETWORK_SLI_TOKEN"}
Oo.prototype.toString=function(){return"InjectionToken("+this.name+")"};function Po(a){return function(){return new a}}
;var Qo={},Ro=(Qo.WEB_UNPLUGGED="^unplugged/",Qo.WEB_UNPLUGGED_ONBOARDING="^unplugged/",Qo.WEB_UNPLUGGED_OPS="^unplugged/",Qo.WEB_UNPLUGGED_PUBLIC="^unplugged/",Qo.WEB_CREATOR="^creator/",Qo.WEB_KIDS="^kids/",Qo.WEB_EXPERIMENTS="^experiments/",Qo.WEB_MUSIC="^music/",Qo.WEB_REMIX="^music/",Qo.WEB_MUSIC_EMBEDDED_PLAYER="^music/",Qo.WEB_MUSIC_EMBEDDED_PLAYER="^main_app/|^sfv/",Qo);
function So(a){var b=void 0===b?"UNKNOWN_INTERFACE":b;if(1===a.length)return a[0];var c=Ro[b];if(c){var d=new RegExp(c),e=q(a);for(c=e.next();!c.done;c=e.next())if(c=c.value,d.exec(c))return c}var f=[];Object.entries(Ro).forEach(function(g){var h=q(g);g=h.next().value;h=h.next().value;b!==g&&f.push(h)});
d=new RegExp(f.join("|"));a.sort(function(g,h){return g.length-h.length});
e=q(a);for(c=e.next();!c.done;c=e.next())if(c=c.value,!d.exec(c))return c;return a[0]}
;function To(a,b){return{method:void 0===b?"POST":b,mode:mi(a)?"same-origin":"cors",credentials:mi(a)?"same-origin":"include"}}
;function Uo(){}
Uo.prototype.o=function(a,b,c){b=void 0===b?{}:b;c=void 0===c?Bo:c;var d;var e=a.clickTrackingParams,f=this.l,g=!1;g=void 0===g?!1:g;f=void 0===f?!1:f;var h=B("INNERTUBE_CONTEXT");if(h){h=tb(h);K("web_no_tracking_params_in_shell_killswitch")||delete h.clickTracking;var k,l;h.client||(h.client={});var n=h.client;"MWEB"===n.clientName&&(n.clientFormFactor=B("IS_TABLET")?"LARGE_FORM_FACTOR":"SMALL_FORM_FACTOR");n.screenWidthPoints=window.innerWidth;n.screenHeightPoints=window.innerHeight;n.screenPixelDensity=
Math.round(window.devicePixelRatio||1);n.screenDensityFloat=window.devicePixelRatio||1;n.utcOffsetMinutes=-Math.floor((new Date).getTimezoneOffset());var t=void 0===t?!1:t;Fj.getInstance();var z="USER_INTERFACE_THEME_LIGHT";Ij(165)?z="USER_INTERFACE_THEME_DARK":Ij(174)?z="USER_INTERFACE_THEME_LIGHT":!K("kevlar_legacy_browsers")&&window.matchMedia&&window.matchMedia("(prefers-color-scheme)").matches&&window.matchMedia("(prefers-color-scheme: dark)").matches&&(z="USER_INTERFACE_THEME_DARK");t=t?z:Eo()||
z;n.userInterfaceTheme=t;if(!g){if(t=Pj())n.connectionType=t;K("web_log_effective_connection_type")&&(t=Qj())&&(h.client.effectiveConnectionType=t)}K("web_log_memory_total_kbytes")&&(null===(k=x.navigator)||void 0===k?0:k.deviceMemory)&&(k=null===(l=x.navigator)||void 0===l?void 0:l.deviceMemory,h.client.memoryTotalKbytes=""+1E6*k);l=ki(x.location.href);!K("web_populate_internal_geo_killswitch")&&l.internalcountrycode&&(n.internalGeo=l.internalcountrycode);"MWEB"===n.clientName||"WEB"===n.clientName?
(n.mainAppWebInfo={graftUrl:x.location.href},K("kevlar_woffle")&&yj.h&&(n.mainAppWebInfo.pwaInstallabilityStatus=yj.h.h?"PWA_INSTALLABILITY_STATUS_CAN_BE_INSTALLED":"PWA_INSTALLABILITY_STATUS_UNKNOWN"),n.mainAppWebInfo.webDisplayMode=zj(),n.mainAppWebInfo.isWebNativeShareAvailable=navigator&&void 0!==navigator.share):"TVHTML5"===n.clientName&&(!K("web_lr_app_quality_killswitch")&&(l=B("LIVING_ROOM_APP_QUALITY"))&&(n.tvAppInfo=Object.assign(n.tvAppInfo||{},{appQuality:l})),l=B("LIVING_ROOM_CERTIFICATION_SCOPE"))&&
(n.tvAppInfo=Object.assign(n.tvAppInfo||{},{certificationScope:l}));if(!K("web_populate_time_zone_itc_killswitch")){b:{if("undefined"!==typeof Intl)try{var u=(new Intl.DateTimeFormat).resolvedOptions().timeZone;break b}catch(ua){}u=void 0}u&&(n.timeZone=u)}(u=sh())?n.experimentsToken=u:delete n.experimentsToken;u=th();Go.h||(Go.h=new Go);n=Go.h.h;l=[];k=0;for(var D in n)l[k++]=n[D];h.request=Object.assign(Object.assign({},h.request),{internalExperimentFlags:u,consistencyTokenJars:l});!K("web_prequest_context_killswitch")&&
(D=B("INNERTUBE_CONTEXT_PREQUEST_CONTEXT"))&&(h.request.externalPrequestContext=D);u=Fj.getInstance();D=Ij(58);u=u.get("gsml","");h.user=Object.assign({},h.user);D&&(h.user.enableSafetyMode=D);u&&(h.user.lockedSafetyMode=!0);K("warm_op_csn_cleanup")?f&&(g=bo())&&(h.clientScreenNonce=g):!g&&(g=bo())&&(h.clientScreenNonce=g);e&&(h.clickTracking={clickTrackingParams:e});if(e=A("yt.mdx.remote.remoteClient_"))h.remoteClient=e;K("web_enable_client_location_service")&&Jo.getInstance().setLocationOnInnerTubeContext(h);
try{var E=ni(void 0),L=E.bid;delete E.bid;h.adSignalsInfo={params:[],bid:L};for(var P=q(Object.entries(E)),S=P.next();!S.done;S=P.next()){var W=q(S.value),Z=W.next().value,Cd=W.next().value;E=Z;L=Cd;null===(d=h.adSignalsInfo.params)||void 0===d?void 0:d.push({key:E,value:""+L})}}catch(ua){Kn(ua)}d=h}else Kn(Error("Error: No InnerTubeContext shell provided in ytconfig.")),d={};d={context:d};if(P=this.h(a)){this.i(d,P,b);var ea,ia;b="/youtubei/v1/"+So(this.j());(a=null===(ia=null===(ea=a.commandMetadata)||
void 0===ea?void 0:ea.webCommandMetadata)||void 0===ia?void 0:ia.apiUrl)&&(b=a);ea=b;(ia=B("INNERTUBE_HOST_OVERRIDE"))&&(ea=String(ia)+String(Zb(ea)));ia={};ia.key=B("INNERTUBE_API_KEY");K("json_condensed_response")&&(ia.prettyPrint="false");ea=li(ea,ia||{},!1);ea={input:ea,sa:To(ea),ba:d,config:Object.assign({},void 0)};ea.config.Ha?ea.config.Ha.identity=c:ea.config.Ha={identity:c};return ea}Kn(new sk("Error: Failed to create Request from Command.",a))};
da.Object.defineProperties(Uo.prototype,{l:{configurable:!0,enumerable:!0,get:function(){return!1}}});function Vo(){}
r(Vo,Uo);Vo.prototype.o=function(){return{input:"/getDatasyncIdsEndpoint",sa:To("/getDatasyncIdsEndpoint","GET"),ba:{}}};
Vo.prototype.j=function(){return[]};
Vo.prototype.h=function(){};
Vo.prototype.i=function(){};var Wo={},Xo=(Wo.GET_DATASYNC_IDS=Po(Vo),Wo);function Yo(a){var b=Fa.apply(1,arguments);if(!Zo(a)||b.some(function(e){return!Zo(e)}))throw Error("Only objects may be merged.");
var c=a;b=q(b);for(var d=b.next();!d.done;d=b.next())$o(c,d.value);return c}
function $o(a,b){for(var c in b)if(Zo(b[c])){if(c in a&&!Zo(a[c]))throw Error("Cannot merge an object into a non-object.");c in a||(a[c]={});$o(a[c],b[c])}else if(ap(b[c])){if(c in a&&!ap(a[c]))throw Error("Cannot merge an array into a non-array.");c in a||(a[c]=[]);bp(a[c],b[c])}else a[c]=b[c];return a}
function bp(a,b){b=q(b);for(var c=b.next();!c.done;c=b.next())c=c.value,Zo(c)?a.push($o({},c)):ap(c)?a.push(bp([],c)):a.push(c);return a}
function Zo(a){return"object"===typeof a&&!Array.isArray(a)}
function ap(a){return"object"===typeof a&&Array.isArray(a)}
;function cp(a,b){Ul.call(this,1,arguments);this.timer=b}
r(cp,Ul);var dp=new Vl("aft-recorded",cp);var ep=window;function fp(){this.timing={};this.clearResourceTimings=function(){};
this.webkitClearResourceTimings=function(){};
this.mozClearResourceTimings=function(){};
this.msClearResourceTimings=function(){};
this.oClearResourceTimings=function(){}}
var Q=ep.performance||ep.mozPerformance||ep.msPerformance||ep.webkitPerformance||new fp;var gp=!1,hp={'script[name="scheduler/scheduler"]':"sj",'script[name="player/base"]':"pj",'link[rel="stylesheet"][name="www-player"]':"pc",'link[rel="stylesheet"][name="player/www-player"]':"pc",'script[name="desktop_polymer/desktop_polymer"]':"dpj",'link[rel="import"][name="desktop_polymer"]':"dph",'script[name="mobile-c3"]':"mcj",'link[rel="stylesheet"][name="mobile-c3"]':"mcc",'script[name="player-plasma-ias-phone/base"]':"mcppj",'script[name="player-plasma-ias-tablet/base"]':"mcptj",'link[rel="stylesheet"][name="mobile-polymer-player-ias"]':"mcpc",
'link[rel="stylesheet"][name="mobile-polymer-player-svg-ias"]':"mcpsc",'script[name="mobile_blazer_core_mod"]':"mbcj",'link[rel="stylesheet"][name="mobile_blazer_css"]':"mbc",'script[name="mobile_blazer_logged_in_users_mod"]':"mbliuj",'script[name="mobile_blazer_logged_out_users_mod"]':"mblouj",'script[name="mobile_blazer_noncore_mod"]':"mbnj","#player_css":"mbpc",'script[name="mobile_blazer_desktopplayer_mod"]':"mbpj",'link[rel="stylesheet"][name="mobile_blazer_tablet_css"]':"mbtc",'script[name="mobile_blazer_watch_mod"]':"mbwj"},
ip=Ua(Q.clearResourceTimings||Q.webkitClearResourceTimings||Q.mozClearResourceTimings||Q.msClearResourceTimings||Q.oClearResourceTimings||Ka,Q);function jp(a){var b=kp(a);if(b.aft)return b.aft;a=B((a||"")+"TIMING_AFT_KEYS",["ol"]);for(var c=a.length,d=0;d<c;d++){var e=b[a[d]];if(e)return e}return NaN}
function lp(){var a;if(K("csi_use_performance_navigation_timing")){var b,c,d,e=null===(d=null===(c=null===(b=null===(a=null===Q||void 0===Q?void 0:Q.getEntriesByType)||void 0===a?void 0:a.call(Q,"navigation"))||void 0===b?void 0:b[0])||void 0===c?void 0:c.toJSON)||void 0===d?void 0:d.call(c);e?(e.requestStart=mp(e.requestStart),e.responseEnd=mp(e.responseEnd),e.redirectStart=mp(e.redirectStart),e.redirectEnd=mp(e.redirectEnd),e.domainLookupEnd=mp(e.domainLookupEnd),e.connectStart=mp(e.connectStart),
e.connectEnd=mp(e.connectEnd),e.responseStart=mp(e.responseStart),e.secureConnectionStart=mp(e.secureConnectionStart),e.domainLookupStart=mp(e.domainLookupStart),e.isPerformanceNavigationTiming=!0,a=e):a=Q.timing}else a=Q.timing;return a}
function np(){return K("csi_use_time_origin")&&Q.timeOrigin?Math.floor(Q.timeOrigin):Q.timing.navigationStart}
function mp(a){return Math.round(np()+a)}
function op(a){y("ytglobal.timing"+(a||"")+"ready_",!0,void 0)}
function pp(a){return A("ytcsi."+(a||"")+"data_")||qp(a)}
function rp(a){a=pp(a);a.info||(a.info={});return a.info}
function kp(a){a=pp(a);a.tick||(a.tick={});return a.tick}
function sp(a){a=pp(a);if(a.gel){var b=a.gel;b.gelInfos||(b.gelInfos={});b.gelTicks||(b.gelTicks={})}else a.gel={gelTicks:{},gelInfos:{}};return a.gel}
function tp(a){a=sp(a);a.gelInfos||(a.gelInfos={});return a.gelInfos}
function up(a){var b=pp(a).nonce;b||(b=Rn(),pp(a).nonce=b);return b}
function qp(a){var b={tick:{},info:{}};y("ytcsi."+(a||"")+"data_",b,void 0);return b}
function vp(a){var b=kp(a||""),c=jp(a);c&&!gp&&($l(dp,new cp(Math.round(c-b._start),a)),gp=!0)}
function wp(a,b){for(var c=q(Object.keys(b)),d=c.next();!d.done;d=c.next())if(d=d.value,!Object.keys(a).includes(d)||"object"===typeof b[d]&&!wp(a[d],b[d]))return!1;return!0}
;function xp(){if(Q.getEntriesByType){var a=Q.getEntriesByType("paint");if(a=hb(a,function(b){return"first-paint"===b.name}))return mp(a.startTime)}a=Q.timing;
return a.cc?Math.max(0,a.cc):0}
;function yp(){var a=A("ytcsi.debug");a||(a=[],y("ytcsi.debug",a,void 0),y("ytcsi.reference",{},void 0));return a}
function zp(a){a=a||"";var b=Ap();if(b[a])return b[a];var c=yp(),d={timerName:a,info:{},tick:{},span:{},jspbInfo:[]};c.push(d);return b[a]=d}
function Ap(){var a=A("ytcsi.reference");if(a)return a;yp();return A("ytcsi.reference")}
;var R={},Bp=(R.auto_search="LATENCY_ACTION_AUTO_SEARCH",R.ad_to_ad="LATENCY_ACTION_AD_TO_AD",R.ad_to_video="LATENCY_ACTION_AD_TO_VIDEO",R["analytics.explore"]="LATENCY_ACTION_CREATOR_ANALYTICS_EXPLORE",R.app_startup="LATENCY_ACTION_APP_STARTUP",R["artist.analytics"]="LATENCY_ACTION_CREATOR_ARTIST_ANALYTICS",R["artist.events"]="LATENCY_ACTION_CREATOR_ARTIST_CONCERTS",R["artist.presskit"]="LATENCY_ACTION_CREATOR_ARTIST_PROFILE",R.browse="LATENCY_ACTION_BROWSE",R.cast_splash="LATENCY_ACTION_CAST_SPLASH",
R.channels="LATENCY_ACTION_CHANNELS",R.creator_channel_dashboard="LATENCY_ACTION_CREATOR_CHANNEL_DASHBOARD",R["channel.analytics"]="LATENCY_ACTION_CREATOR_CHANNEL_ANALYTICS",R["channel.comments"]="LATENCY_ACTION_CREATOR_CHANNEL_COMMENTS",R["channel.content"]="LATENCY_ACTION_CREATOR_POST_LIST",R["channel.copyright"]="LATENCY_ACTION_CREATOR_CHANNEL_COPYRIGHT",R["channel.editing"]="LATENCY_ACTION_CREATOR_CHANNEL_EDITING",R["channel.monetization"]="LATENCY_ACTION_CREATOR_CHANNEL_MONETIZATION",R["channel.music"]=
"LATENCY_ACTION_CREATOR_CHANNEL_MUSIC",R["channel.playlists"]="LATENCY_ACTION_CREATOR_CHANNEL_PLAYLISTS",R["channel.translations"]="LATENCY_ACTION_CREATOR_CHANNEL_TRANSLATIONS",R["channel.videos"]="LATENCY_ACTION_CREATOR_CHANNEL_VIDEOS",R["channel.live_streaming"]="LATENCY_ACTION_CREATOR_LIVE_STREAMING",R.chips="LATENCY_ACTION_CHIPS",R["dialog.copyright_strikes"]="LATENCY_ACTION_CREATOR_DIALOG_COPYRIGHT_STRIKES",R["dialog.uploads"]="LATENCY_ACTION_CREATOR_DIALOG_UPLOADS",R.direct_playback="LATENCY_ACTION_DIRECT_PLAYBACK",
R.embed="LATENCY_ACTION_EMBED",R.entity_key_serialization_perf="LATENCY_ACTION_ENTITY_KEY_SERIALIZATION_PERF",R.entity_key_deserialization_perf="LATENCY_ACTION_ENTITY_KEY_DESERIALIZATION_PERF",R.explore="LATENCY_ACTION_EXPLORE",R.home="LATENCY_ACTION_HOME",R.library="LATENCY_ACTION_LIBRARY",R.live="LATENCY_ACTION_LIVE",R.live_pagination="LATENCY_ACTION_LIVE_PAGINATION",R.onboarding="LATENCY_ACTION_ONBOARDING",R.parent_profile_settings="LATENCY_ACTION_KIDS_PARENT_PROFILE_SETTINGS",R.parent_tools_collection=
"LATENCY_ACTION_PARENT_TOOLS_COLLECTION",R.parent_tools_dashboard="LATENCY_ACTION_PARENT_TOOLS_DASHBOARD",R.player_att="LATENCY_ACTION_PLAYER_ATTESTATION",R["post.comments"]="LATENCY_ACTION_CREATOR_POST_COMMENTS",R["post.edit"]="LATENCY_ACTION_CREATOR_POST_EDIT",R.prebuffer="LATENCY_ACTION_PREBUFFER",R.prefetch="LATENCY_ACTION_PREFETCH",R.profile_settings="LATENCY_ACTION_KIDS_PROFILE_SETTINGS",R.profile_switcher="LATENCY_ACTION_LOGIN",R.reel_watch="LATENCY_ACTION_REEL_WATCH",R.results="LATENCY_ACTION_RESULTS",
R.search_ui="LATENCY_ACTION_SEARCH_UI",R.search_suggest="LATENCY_ACTION_SUGGEST",R.search_zero_state="LATENCY_ACTION_SEARCH_ZERO_STATE",R.secret_code="LATENCY_ACTION_KIDS_SECRET_CODE",R.seek="LATENCY_ACTION_PLAYER_SEEK",R.settings="LATENCY_ACTION_SETTINGS",R.tenx="LATENCY_ACTION_TENX",R.video_to_ad="LATENCY_ACTION_VIDEO_TO_AD",R.watch="LATENCY_ACTION_WATCH",R.watch_it_again="LATENCY_ACTION_KIDS_WATCH_IT_AGAIN",R["watch,watch7"]="LATENCY_ACTION_WATCH",R["watch,watch7_html5"]="LATENCY_ACTION_WATCH",
R["watch,watch7ad"]="LATENCY_ACTION_WATCH",R["watch,watch7ad_html5"]="LATENCY_ACTION_WATCH",R.wn_comments="LATENCY_ACTION_LOAD_COMMENTS",R.ww_rqs="LATENCY_ACTION_WHO_IS_WATCHING",R["video.analytics"]="LATENCY_ACTION_CREATOR_VIDEO_ANALYTICS",R["video.comments"]="LATENCY_ACTION_CREATOR_VIDEO_COMMENTS",R["video.edit"]="LATENCY_ACTION_CREATOR_VIDEO_EDIT",R["video.editor"]="LATENCY_ACTION_CREATOR_VIDEO_VIDEO_EDITOR",R["video.editor_async"]="LATENCY_ACTION_CREATOR_VIDEO_VIDEO_EDITOR_ASYNC",R["video.live_settings"]=
"LATENCY_ACTION_CREATOR_VIDEO_LIVE_SETTINGS",R["video.live_streaming"]="LATENCY_ACTION_CREATOR_VIDEO_LIVE_STREAMING",R["video.monetization"]="LATENCY_ACTION_CREATOR_VIDEO_MONETIZATION",R["video.translations"]="LATENCY_ACTION_CREATOR_VIDEO_TRANSLATIONS",R.voice_assistant="LATENCY_ACTION_VOICE_ASSISTANT",R.cast_load_by_entity_to_watch="LATENCY_ACTION_CAST_LOAD_BY_ENTITY_TO_WATCH",R.networkless_performance="LATENCY_ACTION_NETWORKLESS_PERFORMANCE",R),T={},Cp=(T.ad_allowed="adTypesAllowed",T.yt_abt="adBreakType",
T.ad_cpn="adClientPlaybackNonce",T.ad_docid="adVideoId",T.yt_ad_an="adNetworks",T.ad_at="adType",T.aida="appInstallDataAgeMs",T.browse_id="browseId",T.p="httpProtocol",T.t="transportProtocol",T.cs="commandSource",T.cpn="clientPlaybackNonce",T.ccs="creatorInfo.creatorCanaryState",T.ctop="creatorInfo.topEntityType",T.csn="clientScreenNonce",T.docid="videoId",T.GetHome_rid="requestIds",T.GetSearch_rid="requestIds",T.GetPlayer_rid="requestIds",T.GetWatchNext_rid="requestIds",T.GetBrowse_rid="requestIds",
T.GetLibrary_rid="requestIds",T.is_continuation="isContinuation",T.is_nav="isNavigation",T.b_p="kabukiInfo.browseParams",T.is_prefetch="kabukiInfo.isPrefetch",T.is_secondary_nav="kabukiInfo.isSecondaryNav",T.nav_type="kabukiInfo.navigationType",T.prev_browse_id="kabukiInfo.prevBrowseId",T.query_source="kabukiInfo.querySource",T.voz_type="kabukiInfo.vozType",T.yt_lt="loadType",T.mver="creatorInfo.measurementVersion",T.yt_ad="isMonetized",T.nr="webInfo.navigationReason",T.nrsu="navigationRequestedSameUrl",
T.pnt="performanceNavigationTiming",T.prt="playbackRequiresTap",T.plt="playerInfo.playbackType",T.pis="playerInfo.playerInitializedState",T.paused="playerInfo.isPausedOnLoad",T.yt_pt="playerType",T.fmt="playerInfo.itag",T.yt_pl="watchInfo.isPlaylist",T.yt_pre="playerInfo.preloadType",T.yt_ad_pr="prerollAllowed",T.pa="previousAction",T.yt_red="isRedSubscriber",T.rce="mwebInfo.responseContentEncoding",T.rc="resourceInfo.resourceCache",T.scrh="screenHeight",T.scrw="screenWidth",T.st="serverTimeMs",T.ssdm=
"shellStartupDurationMs",T.br_trs="tvInfo.bedrockTriggerState",T.kebqat="kabukiInfo.earlyBrowseRequestInfo.abandonmentType",T.kebqa="kabukiInfo.earlyBrowseRequestInfo.adopted",T.label="tvInfo.label",T.is_mdx="tvInfo.isMdx",T.preloaded="tvInfo.isPreloaded",T.aac_type="tvInfo.authAccessCredentialType",T.upg_player_vis="playerInfo.visibilityState",T.query="unpluggedInfo.query",T.upg_chip_ids_string="unpluggedInfo.upgChipIdsString",T.yt_vst="videoStreamType",T.vph="viewportHeight",T.vpw="viewportWidth",
T.yt_vis="isVisible",T.rcl="mwebInfo.responseContentLength",T.GetSettings_rid="requestIds",T.GetTrending_rid="requestIds",T.GetMusicSearchSuggestions_rid="requestIds",T.REQUEST_ID="requestIds",T),Dp="isContinuation isNavigation kabukiInfo.earlyBrowseRequestInfo.adopted kabukiInfo.isPrefetch kabukiInfo.isSecondaryNav isMonetized navigationRequestedSameUrl performanceNavigationTiming playerInfo.isPausedOnLoad prerollAllowed isRedSubscriber tvInfo.isMdx tvInfo.isPreloaded isVisible watchInfo.isPlaylist playbackRequiresTap".split(" "),
Ep={},Fp=(Ep.ccs="CANARY_STATE_",Ep.mver="MEASUREMENT_VERSION_",Ep.pis="PLAYER_INITIALIZED_STATE_",Ep.yt_pt="LATENCY_PLAYER_",Ep.pa="LATENCY_ACTION_",Ep.ctop="TOP_ENTITY_TYPE_",Ep.yt_vst="VIDEO_STREAM_TYPE_",Ep),Gp="all_vc ap aq c cbr cbrand cbrver cmodel cos cosver cplatform ctheme cver ei l_an l_mm plid srt yt_fss yt_li vpst vpni2 vpil2 icrc icrt pa GetAccountOverview_rid GetHistory_rid cmt d_vpct d_vpnfi d_vpni nsru pc pfa pfeh pftr pnc prerender psc rc start tcrt tcrc ssr vpr vps yt_abt yt_fn yt_fs yt_pft yt_pre yt_pt yt_pvis ytu_pvis yt_ref yt_sts tds".split(" ");
function Hp(a){return Bp[a]||"LATENCY_ACTION_UNKNOWN"}
function Ip(a,b,c){c=sp(c);if(c.gelInfos)c.gelInfos[a]=!0;else{var d={};c.gelInfos=(d[a]=!0,d)}if(a.match("_rid")){var e=a.split("_rid")[0];a="REQUEST_ID"}if(a in Cp){c=Cp[a];0<=cb(Dp,c)&&(b=!!b);a in Fp&&"string"===typeof b&&(b=Fp[a]+b.toUpperCase());a=b;b=c.split(".");for(var f=d={},g=0;g<b.length-1;g++){var h=b[g];f[h]={};f=f[h]}f[b[b.length-1]]="requestIds"===c?[{id:a,endpoint:e}]:a;return Yo({},d)}0<=cb(Gp,a)||Ln(new sk("Unknown label logged with GEL CSI",a))}
;var U={LATENCY_ACTION_KIDS_PROFILE_SWITCHER:90,LATENCY_ACTION_OFFLINE_THUMBNAIL_TRANSFER:100,LATENCY_ACTION_CREATOR_VIDEO_VIDEO_EDITOR_ASYNC:46,LATENCY_ACTION_CREATOR_VIDEO_VIDEO_EDITOR:37,LATENCY_ACTION_SPINNER_DISPLAYED:14,LATENCY_ACTION_PLAYABILITY_CHECK:10,LATENCY_ACTION_PROCESS:9,LATENCY_ACTION_APP_STARTUP:5,LATENCY_ACTION_ABANDONED_DIRECT_PLAYBACK:154,LATENCY_ACTION_SHARE_VIDEO:153,LATENCY_ACTION_AD_TO_VIDEO_INT:152,LATENCY_ACTION_ABANDONED_BROWSE:151,LATENCY_ACTION_PLAYER_ROTATION:150,LATENCY_ACTION_SHOPPING_IN_APP:124,
LATENCY_ACTION_PLAYER_ATTESTATION:121,LATENCY_ACTION_PLAYER_SEEK:119,LATENCY_ACTION_SUPER_STICKER_BUY_FLOW:114,LATENCY_ACTION_BLOCKS_PERFORMANCE:148,LATENCY_ACTION_ASSISTANT_QUERY:138,LATENCY_ACTION_ASSISTANT_SETTINGS:137,LATENCY_ACTION_ENTITY_KEY_DESERIALIZATION_PERF:129,LATENCY_ACTION_ENTITY_KEY_SERIALIZATION_PERF:128,LATENCY_ACTION_PROOF_OF_ORIGIN_TOKEN_CREATE:127,LATENCY_ACTION_EMBEDS_SDK_INITIALIZATION:123,LATENCY_ACTION_NETWORKLESS_PERFORMANCE:122,LATENCY_ACTION_DOWNLOADS_EXPANSION:133,LATENCY_ACTION_ENTITY_TRANSFORM:131,
LATENCY_ACTION_DOWNLOADS_COMPATIBILITY_LAYER:96,LATENCY_ACTION_EMBEDS_SET_VIDEO:95,LATENCY_ACTION_SETTINGS:93,LATENCY_ACTION_ABANDONED_STARTUP:81,LATENCY_ACTION_MEDIA_BROWSER_ALARM_PLAY:80,LATENCY_ACTION_MEDIA_BROWSER_SEARCH:79,LATENCY_ACTION_MEDIA_BROWSER_LOAD_TREE:78,LATENCY_ACTION_WHO_IS_WATCHING:77,LATENCY_ACTION_CAST_LOAD_BY_ENTITY_TO_WATCH:76,LATENCY_ACTION_LITE_SWITCH_ACCOUNT:73,LATENCY_ACTION_ELEMENTS_PERFORMANCE:70,LATENCY_ACTION_LOCATION_SIGNAL_COLLECTION:69,LATENCY_ACTION_MODIFY_CHANNEL_NOTIFICATION:65,
LATENCY_ACTION_OFFLINE_STORE_START:61,LATENCY_ACTION_REEL_EDITOR:58,LATENCY_ACTION_CHANNEL_SUBSCRIBE:56,LATENCY_ACTION_CHANNEL_PREVIEW:55,LATENCY_ACTION_PREFETCH:52,LATENCY_ACTION_ABANDONED_WATCH:45,LATENCY_ACTION_LOAD_COMMENT_REPLIES:26,LATENCY_ACTION_LOAD_COMMENTS:25,LATENCY_ACTION_EDIT_COMMENT:24,LATENCY_ACTION_NEW_COMMENT:23,LATENCY_ACTION_OFFLINE_SHARING_RECEIVER_PAIRING:19,LATENCY_ACTION_EMBED:18,LATENCY_ACTION_MDX_LAUNCH:15,LATENCY_ACTION_RESOLVE_URL:13,LATENCY_ACTION_CAST_SPLASH:149,LATENCY_ACTION_MDX_CAST:120,
LATENCY_ACTION_MDX_COMMAND:12,LATENCY_ACTION_REEL_SELECT_SEGMENT:136,LATENCY_ACTION_ACCELERATED_EFFECTS:145,LATENCY_ACTION_UPLOAD_AUDIO_MIXER:147,LATENCY_ACTION_SHORTS_SEG_IMP_TRANSCODING:146,LATENCY_ACTION_SHORTS_AUDIO_PICKER_PLAYBACK:130,LATENCY_ACTION_SHORTS_WAVEFORM_DOWNLOAD:125,LATENCY_ACTION_SHORTS_VIDEO_INGESTION:155,LATENCY_ACTION_SHORTS_GALLERY:107,LATENCY_ACTION_SHORTS_TRIM:105,LATENCY_ACTION_SHORTS_EDIT:104,LATENCY_ACTION_SHORTS_CAMERA:103,LATENCY_ACTION_PARENT_TOOLS_DASHBOARD:102,LATENCY_ACTION_PARENT_TOOLS_COLLECTION:101,
LATENCY_ACTION_MUSIC_LOAD_RECOMMENDED_MEDIA_ITEMS:116,LATENCY_ACTION_MUSIC_LOAD_MEDIA_ITEMS:115,LATENCY_ACTION_MUSIC_ALBUM_DETAIL:72,LATENCY_ACTION_MUSIC_PLAYLIST_DETAIL:71,LATENCY_ACTION_CHIPS:68,LATENCY_ACTION_SEARCH_ZERO_STATE:67,LATENCY_ACTION_LIVE_PAGINATION:117,LATENCY_ACTION_LIVE:20,LATENCY_ACTION_PREBUFFER:40,LATENCY_ACTION_TENX:39,LATENCY_ACTION_KIDS_PROFILE_SETTINGS:94,LATENCY_ACTION_KIDS_WATCH_IT_AGAIN:92,LATENCY_ACTION_KIDS_SECRET_CODE:91,LATENCY_ACTION_KIDS_PARENT_PROFILE_SETTINGS:89,
LATENCY_ACTION_KIDS_ONBOARDING:88,LATENCY_ACTION_KIDS_VOICE_SEARCH:82,LATENCY_ACTION_KIDS_CURATED_COLLECTION:62,LATENCY_ACTION_KIDS_LIBRARY:53,LATENCY_ACTION_CREATOR_VIDEO_TRANSLATIONS:38,LATENCY_ACTION_CREATOR_VIDEO_MONETIZATION:74,LATENCY_ACTION_CREATOR_VIDEO_LIVE_STREAMING:141,LATENCY_ACTION_CREATOR_VIDEO_LIVE_SETTINGS:142,LATENCY_ACTION_CREATOR_VIDEO_EDITOR_ASYNC:51,LATENCY_ACTION_CREATOR_VIDEO_EDITOR:50,LATENCY_ACTION_CREATOR_VIDEO_EDIT:36,LATENCY_ACTION_CREATOR_VIDEO_COMMENTS:34,LATENCY_ACTION_CREATOR_VIDEO_ANALYTICS:33,
LATENCY_ACTION_CREATOR_POST_LIST:112,LATENCY_ACTION_CREATOR_POST_EDIT:110,LATENCY_ACTION_CREATOR_POST_COMMENTS:111,LATENCY_ACTION_CREATOR_LIVE_STREAMING:108,LATENCY_ACTION_CREATOR_DIALOG_UPLOADS:86,LATENCY_ACTION_CREATOR_DIALOG_COPYRIGHT_STRIKES:87,LATENCY_ACTION_CREATOR_CHANNEL_VIDEOS:32,LATENCY_ACTION_CREATOR_CHANNEL_TRANSLATIONS:48,LATENCY_ACTION_CREATOR_CHANNEL_PLAYLISTS:139,LATENCY_ACTION_CREATOR_CHANNEL_MUSIC:99,LATENCY_ACTION_CREATOR_CHANNEL_MONETIZATION:43,LATENCY_ACTION_CREATOR_CHANNEL_EDITING:113,
LATENCY_ACTION_CREATOR_CHANNEL_DASHBOARD:49,LATENCY_ACTION_CREATOR_CHANNEL_COPYRIGHT:44,LATENCY_ACTION_CREATOR_CHANNEL_COMMENTS:66,LATENCY_ACTION_CREATOR_CHANNEL_ANALYTICS:31,LATENCY_ACTION_CREATOR_ARTIST_PROFILE:85,LATENCY_ACTION_CREATOR_ARTIST_CONCERTS:84,LATENCY_ACTION_CREATOR_ARTIST_ANALYTICS:83,LATENCY_ACTION_CREATOR_ANALYTICS_EXPLORE:140,LATENCY_ACTION_STORYBOARD_THUMBNAILS:118,LATENCY_ACTION_SEARCH_THUMBNAILS:59,LATENCY_ACTION_ON_DEVICE_MODEL_DOWNLOAD:54,LATENCY_ACTION_VOICE_ASSISTANT:47,LATENCY_ACTION_SEARCH_UI:35,
LATENCY_ACTION_SUGGEST:30,LATENCY_ACTION_AUTO_SEARCH:126,LATENCY_ACTION_DOWNLOADS:98,LATENCY_ACTION_EXPLORE:75,LATENCY_ACTION_VIDEO_LIST:63,LATENCY_ACTION_HOME_RESUME:60,LATENCY_ACTION_SUBSCRIPTIONS_LIST:57,LATENCY_ACTION_THUMBNAIL_LOAD:42,LATENCY_ACTION_FIRST_THUMBNAIL_LOAD:29,LATENCY_ACTION_SUBSCRIPTIONS_FEED:109,LATENCY_ACTION_SUBSCRIPTIONS:28,LATENCY_ACTION_TRENDING:27,LATENCY_ACTION_LIBRARY:21,LATENCY_ACTION_VIDEO_THUMBNAIL:8,LATENCY_ACTION_SHOW_MORE:7,LATENCY_ACTION_VIDEO_PREVIEW:6,LATENCY_ACTION_PREBUFFER_VIDEO:144,
LATENCY_ACTION_PREFETCH_VIDEO:143,LATENCY_ACTION_DIRECT_PLAYBACK:132,LATENCY_ACTION_REEL_WATCH:41,LATENCY_ACTION_AD_TO_AD:22,LATENCY_ACTION_VIDEO_TO_AD:17,LATENCY_ACTION_AD_TO_VIDEO:16,LATENCY_ACTION_ONBOARDING:135,LATENCY_ACTION_LOGIN:97,LATENCY_ACTION_BROWSE:11,LATENCY_ACTION_CHANNELS:4,LATENCY_ACTION_WATCH:3,LATENCY_ACTION_RESULTS:2,LATENCY_ACTION_HOME:1,LATENCY_ACTION_STARTUP:106,LATENCY_ACTION_UNKNOWN:0};U[U.LATENCY_ACTION_KIDS_PROFILE_SWITCHER]="LATENCY_ACTION_KIDS_PROFILE_SWITCHER";
U[U.LATENCY_ACTION_OFFLINE_THUMBNAIL_TRANSFER]="LATENCY_ACTION_OFFLINE_THUMBNAIL_TRANSFER";U[U.LATENCY_ACTION_CREATOR_VIDEO_VIDEO_EDITOR_ASYNC]="LATENCY_ACTION_CREATOR_VIDEO_VIDEO_EDITOR_ASYNC";U[U.LATENCY_ACTION_CREATOR_VIDEO_VIDEO_EDITOR]="LATENCY_ACTION_CREATOR_VIDEO_VIDEO_EDITOR";U[U.LATENCY_ACTION_SPINNER_DISPLAYED]="LATENCY_ACTION_SPINNER_DISPLAYED";U[U.LATENCY_ACTION_PLAYABILITY_CHECK]="LATENCY_ACTION_PLAYABILITY_CHECK";U[U.LATENCY_ACTION_PROCESS]="LATENCY_ACTION_PROCESS";
U[U.LATENCY_ACTION_APP_STARTUP]="LATENCY_ACTION_APP_STARTUP";U[U.LATENCY_ACTION_ABANDONED_DIRECT_PLAYBACK]="LATENCY_ACTION_ABANDONED_DIRECT_PLAYBACK";U[U.LATENCY_ACTION_SHARE_VIDEO]="LATENCY_ACTION_SHARE_VIDEO";U[U.LATENCY_ACTION_AD_TO_VIDEO_INT]="LATENCY_ACTION_AD_TO_VIDEO_INT";U[U.LATENCY_ACTION_ABANDONED_BROWSE]="LATENCY_ACTION_ABANDONED_BROWSE";U[U.LATENCY_ACTION_PLAYER_ROTATION]="LATENCY_ACTION_PLAYER_ROTATION";U[U.LATENCY_ACTION_SHOPPING_IN_APP]="LATENCY_ACTION_SHOPPING_IN_APP";
U[U.LATENCY_ACTION_PLAYER_ATTESTATION]="LATENCY_ACTION_PLAYER_ATTESTATION";U[U.LATENCY_ACTION_PLAYER_SEEK]="LATENCY_ACTION_PLAYER_SEEK";U[U.LATENCY_ACTION_SUPER_STICKER_BUY_FLOW]="LATENCY_ACTION_SUPER_STICKER_BUY_FLOW";U[U.LATENCY_ACTION_BLOCKS_PERFORMANCE]="LATENCY_ACTION_BLOCKS_PERFORMANCE";U[U.LATENCY_ACTION_ASSISTANT_QUERY]="LATENCY_ACTION_ASSISTANT_QUERY";U[U.LATENCY_ACTION_ASSISTANT_SETTINGS]="LATENCY_ACTION_ASSISTANT_SETTINGS";U[U.LATENCY_ACTION_ENTITY_KEY_DESERIALIZATION_PERF]="LATENCY_ACTION_ENTITY_KEY_DESERIALIZATION_PERF";
U[U.LATENCY_ACTION_ENTITY_KEY_SERIALIZATION_PERF]="LATENCY_ACTION_ENTITY_KEY_SERIALIZATION_PERF";U[U.LATENCY_ACTION_PROOF_OF_ORIGIN_TOKEN_CREATE]="LATENCY_ACTION_PROOF_OF_ORIGIN_TOKEN_CREATE";U[U.LATENCY_ACTION_EMBEDS_SDK_INITIALIZATION]="LATENCY_ACTION_EMBEDS_SDK_INITIALIZATION";U[U.LATENCY_ACTION_NETWORKLESS_PERFORMANCE]="LATENCY_ACTION_NETWORKLESS_PERFORMANCE";U[U.LATENCY_ACTION_DOWNLOADS_EXPANSION]="LATENCY_ACTION_DOWNLOADS_EXPANSION";U[U.LATENCY_ACTION_ENTITY_TRANSFORM]="LATENCY_ACTION_ENTITY_TRANSFORM";
U[U.LATENCY_ACTION_DOWNLOADS_COMPATIBILITY_LAYER]="LATENCY_ACTION_DOWNLOADS_COMPATIBILITY_LAYER";U[U.LATENCY_ACTION_EMBEDS_SET_VIDEO]="LATENCY_ACTION_EMBEDS_SET_VIDEO";U[U.LATENCY_ACTION_SETTINGS]="LATENCY_ACTION_SETTINGS";U[U.LATENCY_ACTION_ABANDONED_STARTUP]="LATENCY_ACTION_ABANDONED_STARTUP";U[U.LATENCY_ACTION_MEDIA_BROWSER_ALARM_PLAY]="LATENCY_ACTION_MEDIA_BROWSER_ALARM_PLAY";U[U.LATENCY_ACTION_MEDIA_BROWSER_SEARCH]="LATENCY_ACTION_MEDIA_BROWSER_SEARCH";
U[U.LATENCY_ACTION_MEDIA_BROWSER_LOAD_TREE]="LATENCY_ACTION_MEDIA_BROWSER_LOAD_TREE";U[U.LATENCY_ACTION_WHO_IS_WATCHING]="LATENCY_ACTION_WHO_IS_WATCHING";U[U.LATENCY_ACTION_CAST_LOAD_BY_ENTITY_TO_WATCH]="LATENCY_ACTION_CAST_LOAD_BY_ENTITY_TO_WATCH";U[U.LATENCY_ACTION_LITE_SWITCH_ACCOUNT]="LATENCY_ACTION_LITE_SWITCH_ACCOUNT";U[U.LATENCY_ACTION_ELEMENTS_PERFORMANCE]="LATENCY_ACTION_ELEMENTS_PERFORMANCE";U[U.LATENCY_ACTION_LOCATION_SIGNAL_COLLECTION]="LATENCY_ACTION_LOCATION_SIGNAL_COLLECTION";
U[U.LATENCY_ACTION_MODIFY_CHANNEL_NOTIFICATION]="LATENCY_ACTION_MODIFY_CHANNEL_NOTIFICATION";U[U.LATENCY_ACTION_OFFLINE_STORE_START]="LATENCY_ACTION_OFFLINE_STORE_START";U[U.LATENCY_ACTION_REEL_EDITOR]="LATENCY_ACTION_REEL_EDITOR";U[U.LATENCY_ACTION_CHANNEL_SUBSCRIBE]="LATENCY_ACTION_CHANNEL_SUBSCRIBE";U[U.LATENCY_ACTION_CHANNEL_PREVIEW]="LATENCY_ACTION_CHANNEL_PREVIEW";U[U.LATENCY_ACTION_PREFETCH]="LATENCY_ACTION_PREFETCH";U[U.LATENCY_ACTION_ABANDONED_WATCH]="LATENCY_ACTION_ABANDONED_WATCH";
U[U.LATENCY_ACTION_LOAD_COMMENT_REPLIES]="LATENCY_ACTION_LOAD_COMMENT_REPLIES";U[U.LATENCY_ACTION_LOAD_COMMENTS]="LATENCY_ACTION_LOAD_COMMENTS";U[U.LATENCY_ACTION_EDIT_COMMENT]="LATENCY_ACTION_EDIT_COMMENT";U[U.LATENCY_ACTION_NEW_COMMENT]="LATENCY_ACTION_NEW_COMMENT";U[U.LATENCY_ACTION_OFFLINE_SHARING_RECEIVER_PAIRING]="LATENCY_ACTION_OFFLINE_SHARING_RECEIVER_PAIRING";U[U.LATENCY_ACTION_EMBED]="LATENCY_ACTION_EMBED";U[U.LATENCY_ACTION_MDX_LAUNCH]="LATENCY_ACTION_MDX_LAUNCH";
U[U.LATENCY_ACTION_RESOLVE_URL]="LATENCY_ACTION_RESOLVE_URL";U[U.LATENCY_ACTION_CAST_SPLASH]="LATENCY_ACTION_CAST_SPLASH";U[U.LATENCY_ACTION_MDX_CAST]="LATENCY_ACTION_MDX_CAST";U[U.LATENCY_ACTION_MDX_COMMAND]="LATENCY_ACTION_MDX_COMMAND";U[U.LATENCY_ACTION_REEL_SELECT_SEGMENT]="LATENCY_ACTION_REEL_SELECT_SEGMENT";U[U.LATENCY_ACTION_ACCELERATED_EFFECTS]="LATENCY_ACTION_ACCELERATED_EFFECTS";U[U.LATENCY_ACTION_UPLOAD_AUDIO_MIXER]="LATENCY_ACTION_UPLOAD_AUDIO_MIXER";
U[U.LATENCY_ACTION_SHORTS_SEG_IMP_TRANSCODING]="LATENCY_ACTION_SHORTS_SEG_IMP_TRANSCODING";U[U.LATENCY_ACTION_SHORTS_AUDIO_PICKER_PLAYBACK]="LATENCY_ACTION_SHORTS_AUDIO_PICKER_PLAYBACK";U[U.LATENCY_ACTION_SHORTS_WAVEFORM_DOWNLOAD]="LATENCY_ACTION_SHORTS_WAVEFORM_DOWNLOAD";U[U.LATENCY_ACTION_SHORTS_VIDEO_INGESTION]="LATENCY_ACTION_SHORTS_VIDEO_INGESTION";U[U.LATENCY_ACTION_SHORTS_GALLERY]="LATENCY_ACTION_SHORTS_GALLERY";U[U.LATENCY_ACTION_SHORTS_TRIM]="LATENCY_ACTION_SHORTS_TRIM";
U[U.LATENCY_ACTION_SHORTS_EDIT]="LATENCY_ACTION_SHORTS_EDIT";U[U.LATENCY_ACTION_SHORTS_CAMERA]="LATENCY_ACTION_SHORTS_CAMERA";U[U.LATENCY_ACTION_PARENT_TOOLS_DASHBOARD]="LATENCY_ACTION_PARENT_TOOLS_DASHBOARD";U[U.LATENCY_ACTION_PARENT_TOOLS_COLLECTION]="LATENCY_ACTION_PARENT_TOOLS_COLLECTION";U[U.LATENCY_ACTION_MUSIC_LOAD_RECOMMENDED_MEDIA_ITEMS]="LATENCY_ACTION_MUSIC_LOAD_RECOMMENDED_MEDIA_ITEMS";U[U.LATENCY_ACTION_MUSIC_LOAD_MEDIA_ITEMS]="LATENCY_ACTION_MUSIC_LOAD_MEDIA_ITEMS";
U[U.LATENCY_ACTION_MUSIC_ALBUM_DETAIL]="LATENCY_ACTION_MUSIC_ALBUM_DETAIL";U[U.LATENCY_ACTION_MUSIC_PLAYLIST_DETAIL]="LATENCY_ACTION_MUSIC_PLAYLIST_DETAIL";U[U.LATENCY_ACTION_CHIPS]="LATENCY_ACTION_CHIPS";U[U.LATENCY_ACTION_SEARCH_ZERO_STATE]="LATENCY_ACTION_SEARCH_ZERO_STATE";U[U.LATENCY_ACTION_LIVE_PAGINATION]="LATENCY_ACTION_LIVE_PAGINATION";U[U.LATENCY_ACTION_LIVE]="LATENCY_ACTION_LIVE";U[U.LATENCY_ACTION_PREBUFFER]="LATENCY_ACTION_PREBUFFER";U[U.LATENCY_ACTION_TENX]="LATENCY_ACTION_TENX";
U[U.LATENCY_ACTION_KIDS_PROFILE_SETTINGS]="LATENCY_ACTION_KIDS_PROFILE_SETTINGS";U[U.LATENCY_ACTION_KIDS_WATCH_IT_AGAIN]="LATENCY_ACTION_KIDS_WATCH_IT_AGAIN";U[U.LATENCY_ACTION_KIDS_SECRET_CODE]="LATENCY_ACTION_KIDS_SECRET_CODE";U[U.LATENCY_ACTION_KIDS_PARENT_PROFILE_SETTINGS]="LATENCY_ACTION_KIDS_PARENT_PROFILE_SETTINGS";U[U.LATENCY_ACTION_KIDS_ONBOARDING]="LATENCY_ACTION_KIDS_ONBOARDING";U[U.LATENCY_ACTION_KIDS_VOICE_SEARCH]="LATENCY_ACTION_KIDS_VOICE_SEARCH";
U[U.LATENCY_ACTION_KIDS_CURATED_COLLECTION]="LATENCY_ACTION_KIDS_CURATED_COLLECTION";U[U.LATENCY_ACTION_KIDS_LIBRARY]="LATENCY_ACTION_KIDS_LIBRARY";U[U.LATENCY_ACTION_CREATOR_VIDEO_TRANSLATIONS]="LATENCY_ACTION_CREATOR_VIDEO_TRANSLATIONS";U[U.LATENCY_ACTION_CREATOR_VIDEO_MONETIZATION]="LATENCY_ACTION_CREATOR_VIDEO_MONETIZATION";U[U.LATENCY_ACTION_CREATOR_VIDEO_LIVE_STREAMING]="LATENCY_ACTION_CREATOR_VIDEO_LIVE_STREAMING";U[U.LATENCY_ACTION_CREATOR_VIDEO_LIVE_SETTINGS]="LATENCY_ACTION_CREATOR_VIDEO_LIVE_SETTINGS";
U[U.LATENCY_ACTION_CREATOR_VIDEO_EDITOR_ASYNC]="LATENCY_ACTION_CREATOR_VIDEO_EDITOR_ASYNC";U[U.LATENCY_ACTION_CREATOR_VIDEO_EDITOR]="LATENCY_ACTION_CREATOR_VIDEO_EDITOR";U[U.LATENCY_ACTION_CREATOR_VIDEO_EDIT]="LATENCY_ACTION_CREATOR_VIDEO_EDIT";U[U.LATENCY_ACTION_CREATOR_VIDEO_COMMENTS]="LATENCY_ACTION_CREATOR_VIDEO_COMMENTS";U[U.LATENCY_ACTION_CREATOR_VIDEO_ANALYTICS]="LATENCY_ACTION_CREATOR_VIDEO_ANALYTICS";U[U.LATENCY_ACTION_CREATOR_POST_LIST]="LATENCY_ACTION_CREATOR_POST_LIST";
U[U.LATENCY_ACTION_CREATOR_POST_EDIT]="LATENCY_ACTION_CREATOR_POST_EDIT";U[U.LATENCY_ACTION_CREATOR_POST_COMMENTS]="LATENCY_ACTION_CREATOR_POST_COMMENTS";U[U.LATENCY_ACTION_CREATOR_LIVE_STREAMING]="LATENCY_ACTION_CREATOR_LIVE_STREAMING";U[U.LATENCY_ACTION_CREATOR_DIALOG_UPLOADS]="LATENCY_ACTION_CREATOR_DIALOG_UPLOADS";U[U.LATENCY_ACTION_CREATOR_DIALOG_COPYRIGHT_STRIKES]="LATENCY_ACTION_CREATOR_DIALOG_COPYRIGHT_STRIKES";U[U.LATENCY_ACTION_CREATOR_CHANNEL_VIDEOS]="LATENCY_ACTION_CREATOR_CHANNEL_VIDEOS";
U[U.LATENCY_ACTION_CREATOR_CHANNEL_TRANSLATIONS]="LATENCY_ACTION_CREATOR_CHANNEL_TRANSLATIONS";U[U.LATENCY_ACTION_CREATOR_CHANNEL_PLAYLISTS]="LATENCY_ACTION_CREATOR_CHANNEL_PLAYLISTS";U[U.LATENCY_ACTION_CREATOR_CHANNEL_MUSIC]="LATENCY_ACTION_CREATOR_CHANNEL_MUSIC";U[U.LATENCY_ACTION_CREATOR_CHANNEL_MONETIZATION]="LATENCY_ACTION_CREATOR_CHANNEL_MONETIZATION";U[U.LATENCY_ACTION_CREATOR_CHANNEL_EDITING]="LATENCY_ACTION_CREATOR_CHANNEL_EDITING";U[U.LATENCY_ACTION_CREATOR_CHANNEL_DASHBOARD]="LATENCY_ACTION_CREATOR_CHANNEL_DASHBOARD";
U[U.LATENCY_ACTION_CREATOR_CHANNEL_COPYRIGHT]="LATENCY_ACTION_CREATOR_CHANNEL_COPYRIGHT";U[U.LATENCY_ACTION_CREATOR_CHANNEL_COMMENTS]="LATENCY_ACTION_CREATOR_CHANNEL_COMMENTS";U[U.LATENCY_ACTION_CREATOR_CHANNEL_ANALYTICS]="LATENCY_ACTION_CREATOR_CHANNEL_ANALYTICS";U[U.LATENCY_ACTION_CREATOR_ARTIST_PROFILE]="LATENCY_ACTION_CREATOR_ARTIST_PROFILE";U[U.LATENCY_ACTION_CREATOR_ARTIST_CONCERTS]="LATENCY_ACTION_CREATOR_ARTIST_CONCERTS";U[U.LATENCY_ACTION_CREATOR_ARTIST_ANALYTICS]="LATENCY_ACTION_CREATOR_ARTIST_ANALYTICS";
U[U.LATENCY_ACTION_CREATOR_ANALYTICS_EXPLORE]="LATENCY_ACTION_CREATOR_ANALYTICS_EXPLORE";U[U.LATENCY_ACTION_STORYBOARD_THUMBNAILS]="LATENCY_ACTION_STORYBOARD_THUMBNAILS";U[U.LATENCY_ACTION_SEARCH_THUMBNAILS]="LATENCY_ACTION_SEARCH_THUMBNAILS";U[U.LATENCY_ACTION_ON_DEVICE_MODEL_DOWNLOAD]="LATENCY_ACTION_ON_DEVICE_MODEL_DOWNLOAD";U[U.LATENCY_ACTION_VOICE_ASSISTANT]="LATENCY_ACTION_VOICE_ASSISTANT";U[U.LATENCY_ACTION_SEARCH_UI]="LATENCY_ACTION_SEARCH_UI";U[U.LATENCY_ACTION_SUGGEST]="LATENCY_ACTION_SUGGEST";
U[U.LATENCY_ACTION_AUTO_SEARCH]="LATENCY_ACTION_AUTO_SEARCH";U[U.LATENCY_ACTION_DOWNLOADS]="LATENCY_ACTION_DOWNLOADS";U[U.LATENCY_ACTION_EXPLORE]="LATENCY_ACTION_EXPLORE";U[U.LATENCY_ACTION_VIDEO_LIST]="LATENCY_ACTION_VIDEO_LIST";U[U.LATENCY_ACTION_HOME_RESUME]="LATENCY_ACTION_HOME_RESUME";U[U.LATENCY_ACTION_SUBSCRIPTIONS_LIST]="LATENCY_ACTION_SUBSCRIPTIONS_LIST";U[U.LATENCY_ACTION_THUMBNAIL_LOAD]="LATENCY_ACTION_THUMBNAIL_LOAD";U[U.LATENCY_ACTION_FIRST_THUMBNAIL_LOAD]="LATENCY_ACTION_FIRST_THUMBNAIL_LOAD";
U[U.LATENCY_ACTION_SUBSCRIPTIONS_FEED]="LATENCY_ACTION_SUBSCRIPTIONS_FEED";U[U.LATENCY_ACTION_SUBSCRIPTIONS]="LATENCY_ACTION_SUBSCRIPTIONS";U[U.LATENCY_ACTION_TRENDING]="LATENCY_ACTION_TRENDING";U[U.LATENCY_ACTION_LIBRARY]="LATENCY_ACTION_LIBRARY";U[U.LATENCY_ACTION_VIDEO_THUMBNAIL]="LATENCY_ACTION_VIDEO_THUMBNAIL";U[U.LATENCY_ACTION_SHOW_MORE]="LATENCY_ACTION_SHOW_MORE";U[U.LATENCY_ACTION_VIDEO_PREVIEW]="LATENCY_ACTION_VIDEO_PREVIEW";U[U.LATENCY_ACTION_PREBUFFER_VIDEO]="LATENCY_ACTION_PREBUFFER_VIDEO";
U[U.LATENCY_ACTION_PREFETCH_VIDEO]="LATENCY_ACTION_PREFETCH_VIDEO";U[U.LATENCY_ACTION_DIRECT_PLAYBACK]="LATENCY_ACTION_DIRECT_PLAYBACK";U[U.LATENCY_ACTION_REEL_WATCH]="LATENCY_ACTION_REEL_WATCH";U[U.LATENCY_ACTION_AD_TO_AD]="LATENCY_ACTION_AD_TO_AD";U[U.LATENCY_ACTION_VIDEO_TO_AD]="LATENCY_ACTION_VIDEO_TO_AD";U[U.LATENCY_ACTION_AD_TO_VIDEO]="LATENCY_ACTION_AD_TO_VIDEO";U[U.LATENCY_ACTION_ONBOARDING]="LATENCY_ACTION_ONBOARDING";U[U.LATENCY_ACTION_LOGIN]="LATENCY_ACTION_LOGIN";
U[U.LATENCY_ACTION_BROWSE]="LATENCY_ACTION_BROWSE";U[U.LATENCY_ACTION_CHANNELS]="LATENCY_ACTION_CHANNELS";U[U.LATENCY_ACTION_WATCH]="LATENCY_ACTION_WATCH";U[U.LATENCY_ACTION_RESULTS]="LATENCY_ACTION_RESULTS";U[U.LATENCY_ACTION_HOME]="LATENCY_ACTION_HOME";U[U.LATENCY_ACTION_STARTUP]="LATENCY_ACTION_STARTUP";U[U.LATENCY_ACTION_UNKNOWN]="LATENCY_ACTION_UNKNOWN";var Jp={LATENCY_NETWORK_MOBILE:2,LATENCY_NETWORK_WIFI:1,LATENCY_NETWORK_UNKNOWN:0};Jp[Jp.LATENCY_NETWORK_MOBILE]="LATENCY_NETWORK_MOBILE";
Jp[Jp.LATENCY_NETWORK_WIFI]="LATENCY_NETWORK_WIFI";Jp[Jp.LATENCY_NETWORK_UNKNOWN]="LATENCY_NETWORK_UNKNOWN";var V={CONN_INVALID:31,CONN_CELLULAR_5G_NSA:12,CONN_CELLULAR_5G_SA:11,CONN_WIFI_METERED:10,CONN_CELLULAR_5G:9,CONN_DISCO:8,CONN_CELLULAR_UNKNOWN:7,CONN_CELLULAR_4G:6,CONN_CELLULAR_3G:5,CONN_CELLULAR_2G:4,CONN_WIFI:3,CONN_NONE:2,CONN_UNKNOWN:1,CONN_DEFAULT:0};V[V.CONN_INVALID]="CONN_INVALID";V[V.CONN_CELLULAR_5G_NSA]="CONN_CELLULAR_5G_NSA";V[V.CONN_CELLULAR_5G_SA]="CONN_CELLULAR_5G_SA";
V[V.CONN_WIFI_METERED]="CONN_WIFI_METERED";V[V.CONN_CELLULAR_5G]="CONN_CELLULAR_5G";V[V.CONN_DISCO]="CONN_DISCO";V[V.CONN_CELLULAR_UNKNOWN]="CONN_CELLULAR_UNKNOWN";V[V.CONN_CELLULAR_4G]="CONN_CELLULAR_4G";V[V.CONN_CELLULAR_3G]="CONN_CELLULAR_3G";V[V.CONN_CELLULAR_2G]="CONN_CELLULAR_2G";V[V.CONN_WIFI]="CONN_WIFI";V[V.CONN_NONE]="CONN_NONE";V[V.CONN_UNKNOWN]="CONN_UNKNOWN";V[V.CONN_DEFAULT]="CONN_DEFAULT";
var X={DETAILED_NETWORK_TYPE_NR_NSA:126,DETAILED_NETWORK_TYPE_NR_SA:125,DETAILED_NETWORK_TYPE_INTERNAL_WIFI_IMPAIRED:124,DETAILED_NETWORK_TYPE_APP_WIFI_HOTSPOT:123,DETAILED_NETWORK_TYPE_DISCONNECTED:122,DETAILED_NETWORK_TYPE_NON_MOBILE_UNKNOWN:121,DETAILED_NETWORK_TYPE_MOBILE_UNKNOWN:120,DETAILED_NETWORK_TYPE_WIMAX:119,DETAILED_NETWORK_TYPE_ETHERNET:118,DETAILED_NETWORK_TYPE_BLUETOOTH:117,DETAILED_NETWORK_TYPE_WIFI:116,DETAILED_NETWORK_TYPE_LTE:115,DETAILED_NETWORK_TYPE_HSPAP:114,DETAILED_NETWORK_TYPE_EHRPD:113,
DETAILED_NETWORK_TYPE_EVDO_B:112,DETAILED_NETWORK_TYPE_UMTS:111,DETAILED_NETWORK_TYPE_IDEN:110,DETAILED_NETWORK_TYPE_HSUPA:109,DETAILED_NETWORK_TYPE_HSPA:108,DETAILED_NETWORK_TYPE_HSDPA:107,DETAILED_NETWORK_TYPE_EVDO_A:106,DETAILED_NETWORK_TYPE_EVDO_0:105,DETAILED_NETWORK_TYPE_CDMA:104,DETAILED_NETWORK_TYPE_1_X_RTT:103,DETAILED_NETWORK_TYPE_GPRS:102,DETAILED_NETWORK_TYPE_EDGE:101,DETAILED_NETWORK_TYPE_UNKNOWN:0};X[X.DETAILED_NETWORK_TYPE_NR_NSA]="DETAILED_NETWORK_TYPE_NR_NSA";
X[X.DETAILED_NETWORK_TYPE_NR_SA]="DETAILED_NETWORK_TYPE_NR_SA";X[X.DETAILED_NETWORK_TYPE_INTERNAL_WIFI_IMPAIRED]="DETAILED_NETWORK_TYPE_INTERNAL_WIFI_IMPAIRED";X[X.DETAILED_NETWORK_TYPE_APP_WIFI_HOTSPOT]="DETAILED_NETWORK_TYPE_APP_WIFI_HOTSPOT";X[X.DETAILED_NETWORK_TYPE_DISCONNECTED]="DETAILED_NETWORK_TYPE_DISCONNECTED";X[X.DETAILED_NETWORK_TYPE_NON_MOBILE_UNKNOWN]="DETAILED_NETWORK_TYPE_NON_MOBILE_UNKNOWN";X[X.DETAILED_NETWORK_TYPE_MOBILE_UNKNOWN]="DETAILED_NETWORK_TYPE_MOBILE_UNKNOWN";
X[X.DETAILED_NETWORK_TYPE_WIMAX]="DETAILED_NETWORK_TYPE_WIMAX";X[X.DETAILED_NETWORK_TYPE_ETHERNET]="DETAILED_NETWORK_TYPE_ETHERNET";X[X.DETAILED_NETWORK_TYPE_BLUETOOTH]="DETAILED_NETWORK_TYPE_BLUETOOTH";X[X.DETAILED_NETWORK_TYPE_WIFI]="DETAILED_NETWORK_TYPE_WIFI";X[X.DETAILED_NETWORK_TYPE_LTE]="DETAILED_NETWORK_TYPE_LTE";X[X.DETAILED_NETWORK_TYPE_HSPAP]="DETAILED_NETWORK_TYPE_HSPAP";X[X.DETAILED_NETWORK_TYPE_EHRPD]="DETAILED_NETWORK_TYPE_EHRPD";X[X.DETAILED_NETWORK_TYPE_EVDO_B]="DETAILED_NETWORK_TYPE_EVDO_B";
X[X.DETAILED_NETWORK_TYPE_UMTS]="DETAILED_NETWORK_TYPE_UMTS";X[X.DETAILED_NETWORK_TYPE_IDEN]="DETAILED_NETWORK_TYPE_IDEN";X[X.DETAILED_NETWORK_TYPE_HSUPA]="DETAILED_NETWORK_TYPE_HSUPA";X[X.DETAILED_NETWORK_TYPE_HSPA]="DETAILED_NETWORK_TYPE_HSPA";X[X.DETAILED_NETWORK_TYPE_HSDPA]="DETAILED_NETWORK_TYPE_HSDPA";X[X.DETAILED_NETWORK_TYPE_EVDO_A]="DETAILED_NETWORK_TYPE_EVDO_A";X[X.DETAILED_NETWORK_TYPE_EVDO_0]="DETAILED_NETWORK_TYPE_EVDO_0";X[X.DETAILED_NETWORK_TYPE_CDMA]="DETAILED_NETWORK_TYPE_CDMA";
X[X.DETAILED_NETWORK_TYPE_1_X_RTT]="DETAILED_NETWORK_TYPE_1_X_RTT";X[X.DETAILED_NETWORK_TYPE_GPRS]="DETAILED_NETWORK_TYPE_GPRS";X[X.DETAILED_NETWORK_TYPE_EDGE]="DETAILED_NETWORK_TYPE_EDGE";X[X.DETAILED_NETWORK_TYPE_UNKNOWN]="DETAILED_NETWORK_TYPE_UNKNOWN";var Kp={LATENCY_PLAYER_RTSP:7,LATENCY_PLAYER_HTML5_INLINE:6,LATENCY_PLAYER_HTML5_FULLSCREEN:5,LATENCY_PLAYER_HTML5:4,LATENCY_PLAYER_FRAMEWORK:3,LATENCY_PLAYER_FLASH:2,LATENCY_PLAYER_EXO:1,LATENCY_PLAYER_UNKNOWN:0};Kp[Kp.LATENCY_PLAYER_RTSP]="LATENCY_PLAYER_RTSP";
Kp[Kp.LATENCY_PLAYER_HTML5_INLINE]="LATENCY_PLAYER_HTML5_INLINE";Kp[Kp.LATENCY_PLAYER_HTML5_FULLSCREEN]="LATENCY_PLAYER_HTML5_FULLSCREEN";Kp[Kp.LATENCY_PLAYER_HTML5]="LATENCY_PLAYER_HTML5";Kp[Kp.LATENCY_PLAYER_FRAMEWORK]="LATENCY_PLAYER_FRAMEWORK";Kp[Kp.LATENCY_PLAYER_FLASH]="LATENCY_PLAYER_FLASH";Kp[Kp.LATENCY_PLAYER_EXO]="LATENCY_PLAYER_EXO";Kp[Kp.LATENCY_PLAYER_UNKNOWN]="LATENCY_PLAYER_UNKNOWN";
var Lp={LATENCY_AD_BREAK_TYPE_POSTROLL:3,LATENCY_AD_BREAK_TYPE_MIDROLL:2,LATENCY_AD_BREAK_TYPE_PREROLL:1,LATENCY_AD_BREAK_TYPE_UNKNOWN:0};Lp[Lp.LATENCY_AD_BREAK_TYPE_POSTROLL]="LATENCY_AD_BREAK_TYPE_POSTROLL";Lp[Lp.LATENCY_AD_BREAK_TYPE_MIDROLL]="LATENCY_AD_BREAK_TYPE_MIDROLL";Lp[Lp.LATENCY_AD_BREAK_TYPE_PREROLL]="LATENCY_AD_BREAK_TYPE_PREROLL";Lp[Lp.LATENCY_AD_BREAK_TYPE_UNKNOWN]="LATENCY_AD_BREAK_TYPE_UNKNOWN";var Mp={LATENCY_ACTION_ERROR_STARTUP_TIMEOUT:1,LATENCY_ACTION_ERROR_UNSPECIFIED:0};
Mp[Mp.LATENCY_ACTION_ERROR_STARTUP_TIMEOUT]="LATENCY_ACTION_ERROR_STARTUP_TIMEOUT";Mp[Mp.LATENCY_ACTION_ERROR_UNSPECIFIED]="LATENCY_ACTION_ERROR_UNSPECIFIED";var Np={LIVE_STREAM_MODE_WINDOW:5,LIVE_STREAM_MODE_POST:4,LIVE_STREAM_MODE_LP:3,LIVE_STREAM_MODE_LIVE:2,LIVE_STREAM_MODE_DVR:1,LIVE_STREAM_MODE_UNKNOWN:0};Np[Np.LIVE_STREAM_MODE_WINDOW]="LIVE_STREAM_MODE_WINDOW";Np[Np.LIVE_STREAM_MODE_POST]="LIVE_STREAM_MODE_POST";Np[Np.LIVE_STREAM_MODE_LP]="LIVE_STREAM_MODE_LP";
Np[Np.LIVE_STREAM_MODE_LIVE]="LIVE_STREAM_MODE_LIVE";Np[Np.LIVE_STREAM_MODE_DVR]="LIVE_STREAM_MODE_DVR";Np[Np.LIVE_STREAM_MODE_UNKNOWN]="LIVE_STREAM_MODE_UNKNOWN";var Op={VIDEO_STREAM_TYPE_VOD:3,VIDEO_STREAM_TYPE_DVR:2,VIDEO_STREAM_TYPE_LIVE:1,VIDEO_STREAM_TYPE_UNSPECIFIED:0};Op[Op.VIDEO_STREAM_TYPE_VOD]="VIDEO_STREAM_TYPE_VOD";Op[Op.VIDEO_STREAM_TYPE_DVR]="VIDEO_STREAM_TYPE_DVR";Op[Op.VIDEO_STREAM_TYPE_LIVE]="VIDEO_STREAM_TYPE_LIVE";Op[Op.VIDEO_STREAM_TYPE_UNSPECIFIED]="VIDEO_STREAM_TYPE_UNSPECIFIED";
var Pp={YT_IDB_TRANSACTION_TYPE_READ:2,YT_IDB_TRANSACTION_TYPE_WRITE:1,YT_IDB_TRANSACTION_TYPE_UNKNOWN:0};Pp[Pp.YT_IDB_TRANSACTION_TYPE_READ]="YT_IDB_TRANSACTION_TYPE_READ";Pp[Pp.YT_IDB_TRANSACTION_TYPE_WRITE]="YT_IDB_TRANSACTION_TYPE_WRITE";Pp[Pp.YT_IDB_TRANSACTION_TYPE_UNKNOWN]="YT_IDB_TRANSACTION_TYPE_UNKNOWN";var Qp={PLAYER_ROTATION_TYPE_PORTRAIT_TO_FULLSCREEN:2,PLAYER_ROTATION_TYPE_FULLSCREEN_TO_PORTRAIT:1,PLAYER_ROTATION_TYPE_UNKNOWN:0};Qp[Qp.PLAYER_ROTATION_TYPE_PORTRAIT_TO_FULLSCREEN]="PLAYER_ROTATION_TYPE_PORTRAIT_TO_FULLSCREEN";
Qp[Qp.PLAYER_ROTATION_TYPE_FULLSCREEN_TO_PORTRAIT]="PLAYER_ROTATION_TYPE_FULLSCREEN_TO_PORTRAIT";Qp[Qp.PLAYER_ROTATION_TYPE_UNKNOWN]="PLAYER_ROTATION_TYPE_UNKNOWN";var Rp=x.ytLoggingGelSequenceIdObj_||{};y("ytLoggingGelSequenceIdObj_",Rp,void 0);function Sp(a,b,c){c=void 0===c?{}:c;var d=Math.round(c.timestamp||N());F(a,1,d<Number.MAX_SAFE_INTEGER?d:0);var e=$h();d=new Tg;F(d,1,c.timestamp||!isFinite(e)?-1:e);if(K("log_sequence_info_on_gel_web")&&c.X){e=c.X;var f=xj(e),g=new Sg;F(g,2,f);F(g,1,e);G(d,3,g);c.ub&&delete Rp[c.X]}G(a,33,d);(c.mc?ij:dj)({endpoint:"log_event",payload:a,cttAuthInfo:c.cttAuthInfo,wa:c.wa},b)}
;function Tp(a,b){b=void 0===b?{}:b;var c=!1;B("ytLoggingEventsDefaultDisabled",!1)&&rn===rn&&(c=!0);Sp(a,c?null:rn,b)}
;function Up(a,b,c){var d=new Ug;Sc(d,72,a);c?Sp(d,c,b):Tp(d,b)}
function Vp(a,b,c){var d=new Ug;Sc(d,73,a);c?Sp(d,c,b):Tp(d,b)}
function Wp(a,b,c){var d=new Ug;Sc(d,78,a);c?Sp(d,c,b):Tp(d,b)}
function Xp(a,b,c){var d=new Ug;Sc(d,208,a);c?Sp(d,c,b):Tp(d,b)}
function Yp(a,b,c){var d=new Ug;Sc(d,156,a);c?Sp(d,c,b):Tp(d,b)}
function Zp(a,b,c){var d=new Ug;Sc(d,215,a);c?Sp(d,c,b):Tp(d,b)}
;var $p=x.ytLoggingLatencyUsageStats_||{};y("ytLoggingLatencyUsageStats_",$p,void 0);function aq(){this.h=0}
function bq(){aq.h||(aq.h=new aq);return aq.h}
aq.prototype.tick=function(a,b,c,d){cq(this,"tick_"+a+"_"+b)||(c={timestamp:c,cttAuthInfo:d},K("web_csi_via_jspb")?(d=new Rg,F(d,1,a),F(d,2,b),a=new Ug,Sc(a,5,d),Tp(a,c)):pk("latencyActionTicked",{tickName:a,clientActionNonce:b},c))};
aq.prototype.info=function(a,b,c){var d=Object.keys(a).join("");cq(this,"info_"+d+"_"+b)||(a=Object.assign({},a),a.clientActionNonce=b,pk("latencyActionInfo",a,{cttAuthInfo:c}))};
aq.prototype.jspbInfo=function(a,b,c){for(var d="",e=0;e<a.toJSON().length;e++)void 0!==a.toJSON()[e]&&(d=0===e?d.concat(""+e):d.concat("_"+e));cq(this,"info_"+d+"_"+b)||(F(a,2,b),b={cttAuthInfo:c},c=new Ug,Sc(c,7,a),Tp(c,b))};
aq.prototype.span=function(a,b,c){var d=Object.keys(a).join("");cq(this,"span_"+d+"_"+b)||(a.clientActionNonce=b,pk("latencyActionSpan",a,{cttAuthInfo:c}))};
function cq(a,b){$p[b]=$p[b]||{count:0};var c=$p[b];c.count++;c.time=N();a.h||(a.h=Ph(function(){var d=N(),e;for(e in $p)$p[e]&&6E4<d-$p[e].time&&delete $p[e];a&&(a.h=0)},5E3));
return 5<c.count?(6===c.count&&1>1E5*Math.random()&&(c=new sk("CSI data exceeded logging limit with key",b.split("_")),0<=b.indexOf("plev")||Ln(c)),!0):!1}
;function dq(){var a=["ol"];zp("").info.actionType="embed";a&&oh("TIMING_AFT_KEYS",a);oh("TIMING_ACTION","embed");a=B("TIMING_INFO",{});if(K("web_csi_via_jspb")){var b=new Ng;a=q(Object.entries(a));for(var c=a.next();!c.done;c=a.next()){var d=q(c.value);c=d.next().value;d=d.next().value;switch(c){case "GetBrowse_rid":var e=new Qg;F(e,1,c);F(e,2,String(d));Pg(b,e);break;case "GetGuide_rid":e=new Qg;F(e,1,c);F(e,2,String(d));Pg(b,e);break;case "GetHome_rid":e=new Qg;F(e,1,c);F(e,2,String(d));Pg(b,e);
break;case "GetPlayer_rid":e=new Qg;F(e,1,c);F(e,2,String(d));Pg(b,e);break;case "GetSearch_rid":e=new Qg;F(e,1,c);F(e,2,String(d));Pg(b,e);break;case "GetSettings_rid":e=new Qg;F(e,1,c);F(e,2,String(d));Pg(b,e);break;case "GetTrending_rid":e=new Qg;F(e,1,c);F(e,2,String(d));Pg(b,e);break;case "GetWatchNext_rid":e=new Qg;F(e,1,c);F(e,2,String(d));Pg(b,e);break;case "yt_red":F(b,14,!!d);break;case "yt_ad":F(b,9,!!d)}}eq(b)}else for(b in a)a.hasOwnProperty(b)&&fq(b,a[b]);b={isNavigation:!0,actionType:Hp(ph("TIMING_ACTION"))};
if(a=ph("PREVIOUS_ACTION"))b.previousAction=Hp(a);if(a=B("CLIENT_PROTOCOL"))b.httpProtocol=a;if(a=B("CLIENT_TRANSPORT"))b.transportProtocol=a;(a=bo())&&"UNDEFINED_CSN"!==a&&(b.clientScreenNonce=a);a=gq();if(1===a||-1===a)b.isVisible=!0;a=rp();b.loadType="cold";c=lp();if(d=np())Y("srt",c.responseStart),1!==a.prerender&&Y("_start",d,void 0);a=xp();0<a&&Y("fpt",a);a=lp();a.isPerformanceNavigationTiming&&hq({performanceNavigationTiming:!0},void 0);Y("nreqs",a.requestStart,void 0);Y("nress",a.responseStart,
void 0);Y("nrese",a.responseEnd,void 0);0<a.redirectEnd-a.redirectStart&&(Y("nrs",a.redirectStart,void 0),Y("nre",a.redirectEnd,void 0));0<a.domainLookupEnd-a.domainLookupStart&&(Y("ndnss",a.domainLookupStart,void 0),Y("ndnse",a.domainLookupEnd,void 0));0<a.connectEnd-a.connectStart&&(Y("ntcps",a.connectStart,void 0),Y("ntcpe",a.connectEnd,void 0));a.secureConnectionStart>=np()&&0<a.connectEnd-a.secureConnectionStart&&(Y("nstcps",a.secureConnectionStart,void 0),Y("ntcpe",a.connectEnd,void 0));Q&&
"getEntriesByType"in Q&&iq();a=[];if(document.querySelector&&Q&&Q.getEntriesByName)for(var f in hp)hp.hasOwnProperty(f)&&(c=hp[f],jq(f,c)&&a.push(c));if(0<a.length)for(b.resourceInfo=[],f=q(a),a=f.next();!a.done;a=f.next())b.resourceInfo.push({resourceCache:a.value});hq(b);f=rp();b=tp();if("cold"===f.yt_lt||"cold"===b.loadType){a=kp();c=sp();c=c.gelTicks?c.gelTicks:c.gelTicks={};for(var g in a)g in c||Y(g,a[g]);g={};a=!1;c=q(Object.keys(f));for(d=c.next();!d.done;d=c.next())d=d.value,(d=Ip(d,f[d]))&&
!wp(tp(void 0),d)&&(Yo(b,d),Yo(g,d),a=!0);a&&hq(g)}op();g=ph("TIMING_ACTION");A("ytglobal.timingready_")&&g&&"_start"in kp(void 0)&&jp()&&vp()}
function fq(a,b,c){null!==b&&(rp(c)[a]=b,(a=Ip(a,b,c))&&hq(a,c))}
function hq(a,b){if(K("web_csi_via_jspb")){for(var c=new Ng,d=Object.keys(a),e=Object.values(a),f=0;f<d.length;f++)switch(d[f]){case "actionType":try{F(c,1,U[e[f]])}catch(l){M(Error("Codegen laipb translator failed to set actionType"))}break;case "clientActionNonce":try{F(c,2,e[f])}catch(l){M(Error("Codegen laipb translator failed to set clientActionNonce"))}break;case "clientScreenNonce":try{F(c,4,e[f])}catch(l){M(Error("Codegen laipb translator failed to set clientScreenNonce"))}break;case "actionVisualElement":try{G(c,
88,e[f])}catch(l){M(Error("Codegen laipb translator failed to set actionVisualElement"))}break;case "loadType":try{F(c,3,e[f])}catch(l){M(Error("Codegen laipb translator failed to set loadType"))}break;case "isFirstInstall":try{F(c,55,e[f])}catch(l){M(Error("Codegen laipb translator failed to set isFirstInstall"))}break;case "networkType":try{F(c,5,Jp[e[f]])}catch(l){M(Error("Codegen laipb translator failed to set networkType"))}break;case "connectionType":try{F(c,26,V[e[f]])}catch(l){M(Error("Codegen laipb translator failed to set connectionType"))}break;
case "detailedConnectionType":try{F(c,27,X[e[f]])}catch(l){M(Error("Codegen laipb translator failed to set detailedConnectionType"))}break;case "isVisible":try{F(c,6,e[f])}catch(l){M(Error("Codegen laipb translator failed to set isVisible"))}break;case "playerType":try{F(c,7,Kp[e[f]])}catch(l){M(Error("Codegen laipb translator failed to set playerType"))}break;case "clientPlaybackNonce":try{F(c,8,e[f])}catch(l){M(Error("Codegen laipb translator failed to set clientPlaybackNonce"))}break;case "adClientPlaybackNonce":try{F(c,
28,e[f])}catch(l){M(Error("Codegen laipb translator failed to set adClientPlaybackNonce"))}break;case "previousCpn":try{F(c,77,e[f])}catch(l){M(Error("Codegen laipb translator failed to set previousCpn"))}break;case "targetCpn":try{F(c,76,e[f])}catch(l){M(Error("Codegen laipb translator failed to set targetCpn"))}break;case "isMonetized":try{F(c,9,e[f])}catch(l){M(Error("Codegen laipb translator failed to set isMonetized"))}break;case "isPrerollAllowed":try{F(c,16,e[f])}catch(l){M(Error("Codegen laipb translator failed to set isPrerollAllowed"))}break;
case "isPrerollShown":try{F(c,17,e[f])}catch(l){M(Error("Codegen laipb translator failed to set isPrerollShown"))}break;case "adType":try{F(c,12,e[f])}catch(l){M(Error("Codegen laipb translator failed to set adType"))}break;case "adTypesAllowed":try{F(c,36,e[f])}catch(l){M(Error("Codegen laipb translator failed to set adTypesAllowed"))}break;case "adNetworks":try{F(c,37,e[f])}catch(l){M(Error("Codegen laipb translator failed to set adNetworks"))}break;case "previousAction":try{F(c,13,U[e[f]])}catch(l){M(Error("Codegen laipb translator failed to set previousAction"))}break;
case "isRedSubscriber":try{F(c,14,e[f])}catch(l){M(Error("Codegen laipb translator failed to set isRedSubscriber"))}break;case "serverTimeMs":try{F(c,15,e[f])}catch(l){M(Error("Codegen laipb translator failed to set serverTimeMs"))}break;case "spinnerInfo":try{G(c,18,e[f])}catch(l){M(Error("Codegen laipb translator failed to set spinnerInfo"))}break;case "videoId":try{c.setVideoId(e[f])}catch(l){M(Error("Codegen laipb translator failed to set videoId"))}break;case "adVideoId":try{F(c,20,e[f])}catch(l){M(Error("Codegen laipb translator failed to set adVideoId"))}break;
case "targetVideoId":try{F(c,78,e[f])}catch(l){M(Error("Codegen laipb translator failed to set targetVideoId"))}break;case "adBreakType":try{F(c,21,Lp[e[f]])}catch(l){M(Error("Codegen laipb translator failed to set adBreakType"))}break;case "isNavigation":try{F(c,25,e[f])}catch(l){M(Error("Codegen laipb translator failed to set isNavigation"))}break;case "viewportHeight":try{F(c,29,e[f])}catch(l){M(Error("Codegen laipb translator failed to set viewportHeight"))}break;case "viewportWidth":try{F(c,
30,e[f])}catch(l){M(Error("Codegen laipb translator failed to set viewportWidth"))}break;case "screenHeight":try{F(c,84,e[f])}catch(l){M(Error("Codegen laipb translator failed to set screenHeight"))}break;case "screenWidth":try{F(c,85,e[f])}catch(l){M(Error("Codegen laipb translator failed to set screenWidth"))}break;case "browseId":try{F(c,31,e[f])}catch(l){M(Error("Codegen laipb translator failed to set browseId"))}break;case "isCacheHit":try{F(c,32,e[f])}catch(l){M(Error("Codegen laipb translator failed to set isCacheHit"))}break;
case "httpProtocol":try{F(c,33,e[f])}catch(l){M(Error("Codegen laipb translator failed to set httpProtocol"))}break;case "transportProtocol":try{F(c,34,e[f])}catch(l){M(Error("Codegen laipb translator failed to set transportProtocol"))}break;case "searchQuery":try{F(c,41,e[f])}catch(l){M(Error("Codegen laipb translator failed to set searchQuery"))}break;case "isContinuation":try{F(c,42,e[f])}catch(l){M(Error("Codegen laipb translator failed to set isContinuation"))}break;case "availableProcessors":try{F(c,
43,e[f])}catch(l){M(Error("Codegen laipb translator failed to set availableProcessors"))}break;case "sdk":try{F(c,44,e[f])}catch(l){M(Error("Codegen laipb translator failed to set sdk"))}break;case "isLocalStream":try{F(c,45,e[f])}catch(l){M(Error("Codegen laipb translator failed to set isLocalStream"))}break;case "navigationRequestedSameUrl":try{F(c,64,e[f])}catch(l){M(Error("Codegen laipb translator failed to set navigationRequestedSameUrl"))}break;case "shellStartupDurationMs":try{F(c,70,e[f])}catch(l){M(Error("Codegen laipb translator failed to set shellStartupDurationMs"))}break;
case "appInstallDataAgeMs":try{F(c,73,e[f])}catch(l){M(Error("Codegen laipb translator failed to set appInstallDataAgeMs"))}break;case "latencyActionError":try{F(c,71,Mp[e[f]])}catch(l){M(Error("Codegen laipb translator failed to set latencyActionError"))}break;case "actionStep":try{F(c,79,e[f])}catch(l){M(Error("Codegen laipb translator failed to set actionStep"))}break;case "jsHeapSizeLimit":try{F(c,80,e[f])}catch(l){M(Error("Codegen laipb translator failed to set jsHeapSizeLimit"))}break;case "totalJsHeapSize":try{F(c,
81,e[f])}catch(l){M(Error("Codegen laipb translator failed to set totalJsHeapSize"))}break;case "usedJsHeapSize":try{F(c,82,e[f])}catch(l){M(Error("Codegen laipb translator failed to set usedJsHeapSize"))}break;case "resourceInfo":try{Uc(c,83,Mg,e[f])}catch(l){M(Error("Codegen laipb translator failed to set resourceInfo"))}break;case "sourceVideoDurationMs":try{F(c,90,e[f])}catch(l){M(Error("Codegen laipb translator failed to set sourceVideoDurationMs"))}break;case "playerInfo":try{G(c,22,e[f])}catch(l){M(Error("Codegen laipb translator failed to set playerInfo"))}break;
case "commentInfo":try{G(c,23,e[f])}catch(l){M(Error("Codegen laipb translator failed to set commentInfo"))}break;case "mdxInfo":try{G(c,24,e[f])}catch(l){M(Error("Codegen laipb translator failed to set mdxInfo"))}break;case "watchInfo":try{G(c,35,e[f])}catch(l){M(Error("Codegen laipb translator failed to set watchInfo"))}break;case "adPrebufferedTimeSecs":try{F(c,39,e[f])}catch(l){M(Error("Codegen laipb translator failed to set adPrebufferedTimeSecs"))}break;case "thumbnailLoadInfo":try{G(c,40,e[f])}catch(l){M(Error("Codegen laipb translator failed to set thumbnailLoadInfo"))}break;
case "creatorInfo":try{G(c,46,e[f])}catch(l){M(Error("Codegen laipb translator failed to set creatorInfo"))}break;case "unpluggedInfo":try{G(c,50,e[f])}catch(l){M(Error("Codegen laipb translator failed to set unpluggedInfo"))}break;case "isLivestream":try{F(c,47,e[f])}catch(l){M(Error("Codegen laipb translator failed to set isLivestream"))}break;case "liveStreamMode":try{F(c,91,Np[e[f]])}catch(l){M(Error("Codegen laipb translator failed to set liveStreamMode"))}break;case "adCpn2":try{F(c,48,e[f])}catch(l){M(Error("Codegen laipb translator failed to set adCpn2"))}break;
case "adDaiDriftMillis":try{F(c,49,e[f])}catch(l){M(Error("Codegen laipb translator failed to set adDaiDriftMillis"))}break;case "videoStreamType":try{F(c,53,Op[e[f]])}catch(l){M(Error("Codegen laipb translator failed to set videoStreamType"))}break;case "reelInfo":try{G(c,54,e[f])}catch(l){M(Error("Codegen laipb translator failed to set reelInfo"))}break;case "subscriptionsFeedInfo":try{G(c,72,e[f])}catch(l){M(Error("Codegen laipb translator failed to set subscriptionsFeedInfo"))}break;case "playbackRequiresTap":try{F(c,
56,e[f])}catch(l){M(Error("Codegen laipb translator failed to set playbackRequiresTap"))}break;case "requestIds":try{Pg(c,e[f])}catch(l){M(Error("Codegen laipb translator failed to set requestIds"))}break;case "mediaBrowserActionInfo":try{G(c,58,e[f])}catch(l){M(Error("Codegen laipb translator failed to set mediaBrowserActionInfo"))}break;case "performanceNavigationTiming":try{F(c,67,e[f])}catch(l){M(Error("Codegen laipb translator failed to set performanceNavigationTiming"))}break;case "musicLoadActionInfo":try{G(c,
69,e[f])}catch(l){M(Error("Codegen laipb translator failed to set musicLoadActionInfo"))}break;case "transactionType":try{F(c,74,Pp[e[f]])}catch(l){M(Error("Codegen laipb translator failed to set transactionType"))}break;case "shoppingInfo":try{G(c,75,e[f])}catch(l){M(Error("Codegen laipb translator failed to set shoppingInfo"))}break;case "prefetchInfo":try{G(c,86,e[f])}catch(l){M(Error("Codegen laipb translator failed to set prefetchInfo"))}break;case "accelerationSession":try{G(c,87,e[f])}catch(l){M(Error("Codegen laipb translator failed to set accelerationSession"))}break;
case "playerRotationType":try{F(c,101,Qp[e[f]])}catch(l){M(Error("Codegen laipb translator failed to set playerRotationType"))}break;case "webInfo":try{G(c,38,e[f])}catch(l){M(Error("Codegen laipb translator failed to set webInfo"))}break;case "tvInfo":try{G(c,51,e[f])}catch(l){M(Error("Codegen laipb translator failed to set tvInfo"))}break;case "kabukiInfo":try{G(c,52,e[f])}catch(l){M(Error("Codegen laipb translator failed to set kabukiInfo"))}break;case "mwebInfo":try{G(c,59,e[f])}catch(l){M(Error("Codegen laipb translator failed to set mwebInfo"))}break;
case "musicInfo":try{G(c,65,e[f])}catch(l){M(Error("Codegen laipb translator failed to set musicInfo"))}break;case "allowedPreroll":try{F(c,10,e[f])}catch(l){M(Error("Codegen laipb translator failed to set allowedPreroll"))}break;case "shownPreroll":try{F(c,11,e[f])}catch(l){M(Error("Codegen laipb translator failed to set shownPreroll"))}break;case "getHomeRequestId":try{F(c,57,e[f])}catch(l){M(Error("Codegen laipb translator failed to set getHomeRequestId"))}break;case "getSearchRequestId":try{F(c,
60,e[f])}catch(l){M(Error("Codegen laipb translator failed to set getSearchRequestId"))}break;case "getPlayerRequestId":try{F(c,61,e[f])}catch(l){M(Error("Codegen laipb translator failed to set getPlayerRequestId"))}break;case "getWatchNextRequestId":try{F(c,62,e[f])}catch(l){M(Error("Codegen laipb translator failed to set getWatchNextRequestId"))}break;case "getBrowseRequestId":try{F(c,63,e[f])}catch(l){M(Error("Codegen laipb translator failed to set getBrowseRequestId"))}break;case "getLibraryRequestId":try{F(c,
66,e[f])}catch(l){M(Error("Codegen laipb translator failed to set getLibraryRequestId"))}}eq(c,b)}else{var g=zp(b||"");Yo(g.info,a);Yo(tp(b),a);var h=up(b),k=pp(b).cttAuthInfo;bq().info(a,h,k)}}
function eq(a,b){var c=sp(b);c.jspbInfos||(c.jspbInfos=[]);c.jspbInfos.push(a);zp(b||"").jspbInfo.push(a);c=up(b);b=pp(b).cttAuthInfo;bq().jspbInfo(a,c,b)}
function Y(a,b,c){if(!b&&"_"!==a[0]){var d=a;Q.mark&&(0==d.lastIndexOf("mark_",0)||(d="mark_"+d),c&&(d+=" ("+c+")"),Q.mark(d))}zp(c||"").tick[a]=b||N();d=sp(c);d.gelTicks&&(d.gelTicks[a]=!0);d=kp(c);var e=b||N();d[a]=e;e=up(c);var f=pp(c).cttAuthInfo;if("_start"===a){var g=bq();cq(g,"baseline_"+e)||(b={timestamp:b,cttAuthInfo:f},K("web_csi_via_jspb")?(f=new Lg,F(f,1,e),e=new Ug,Sc(e,6,f),Tp(e,b)):pk("latencyActionBaselined",{clientActionNonce:e},b))}else bq().tick(a,e,b,f);vp(c);return d[a]}
function kq(){var a=up(void 0);requestAnimationFrame(function(){setTimeout(function(){a===up(void 0)&&Y("ol",void 0,void 0)},0)})}
function gq(){var a=document;if("visibilityState"in a)a=a.visibilityState;else{var b=xh+"VisibilityState";a=b in a?a[b]:void 0}switch(a){case "hidden":return 0;case "visible":return 1;case "prerender":return 2;case "unloaded":return 3;default:return-1}}
function jq(a,b){a=document.querySelector(a);if(!a)return!1;var c="",d=a.nodeName;"SCRIPT"===d?(c=a.src,c||(c=a.getAttribute("data-timing-href"))&&(c=window.location.protocol+c)):"LINK"===d&&(c=a.href);Tb()&&a.setAttribute("nonce",Tb());return c?(a=Q.getEntriesByName(c))&&a[0]&&(a=a[0],c=np(),Y("rsf_"+b,c+Math.round(a.fetchStart)),Y("rse_"+b,c+Math.round(a.responseEnd)),void 0!==a.transferSize&&0===a.transferSize)?!0:!1:!1}
function iq(){var a=window.location.protocol,b=Q.getEntriesByType("resource");b=eb(b,function(c){return 0===c.name.indexOf(a+"//fonts.gstatic.com/s/")});
(b=gb(b,function(c,d){return d.duration>c.duration?d:c},{duration:0}))&&0<b.startTime&&0<b.responseEnd&&(Y("wffs",mp(b.startTime)),Y("wffe",mp(b.responseEnd)))}
var lq=window;lq.ytcsi&&(lq.ytcsi.info=fq,lq.ytcsi.tick=Y);var mq=["consistency","mss","client_location","entities","store"];function nq(a,b,c,d){this.o=a;this.J=b;this.l=c;this.j=d;this.i=void 0;this.h=new Map;a.Aa||(a.Aa={});a.Aa=Object.assign(Object.assign({},Xo),a.Aa)}
function oq(a,b,c,d){if(void 0!==nq.h){if(d=nq.h,a=[a!==d.o,b!==d.J,c!==d.l,!1,!1,void 0!==d.i],a.some(function(e){return e}))throw new sk("InnerTubeTransportService is already initialized",a);
}else nq.h=new nq(a,b,c,d)}
function pq(){var a=nq.h,b={signalServiceEndpoint:{signal:"GET_DATASYNC_IDS"}};var c=void 0===c?Bo:c;var d=Mo(b,a.o);if(!d)return If(new sk("Error: No request builder found for command.",b));var e=d.o(b,void 0,c);return e?new Df(function(f){var g,h,k;return w(function(l){if(1==l.h)return h="cors"===(null===(g=e.sa)||void 0===g?void 0:g.mode)?"cors":void 0,v(l,qq(a,e.config,h),2);k=l.i;f(rq(a,e,k));l.h=0})}):If(new sk("Error: Failed to build request for command.",b))}
function rq(a,b,c){var d,e,f,g,h,k,l,n,t,z,u,D,E,L,P,S,W;return w(function(Z){switch(Z.h){case 1:Z.s(2);break;case 3:if((l=Z.i)&&!l.isExpired())return Z.return(Promise.resolve(l.h()));case 2:if((n=null===(d=b.config)||void 0===d?void 0:d.Eo)&&a.h.has(n)&&K("web_memoize_inflight_requests"))return Z.return(a.h.get(n));if(null===(e=null===b||void 0===b?void 0:b.ba)||void 0===e?0:e.context)for(t=q([]),z=t.next();!z.done;z=t.next())u=z.value,u.Co(b.ba.context);if(null===(f=a.i)||void 0===f?0:f.l(b.input,
b.ba))return Z.return(a.i.j(b.input,b.ba));D=JSON.stringify(b.ba);b.sa=Object.assign(Object.assign({},b.sa),{headers:c});E=Object.assign({},b.sa);"POST"===b.sa.method&&(E=Object.assign(Object.assign({},E),{body:D}));(null===(g=b.config)||void 0===g?0:g.ic)&&Y(b.config.ic);L=a.J.fetch(b.input,E,b.config);n&&a.h.set(n,L);return v(Z,L,4);case 4:P=Z.i;n&&a.h.has(n)&&a.h.delete(n);(null===(h=b.config)||void 0===h?0:h.jc)&&Y(b.config.jc);if(P||null===(k=a.i)||void 0===k||!k.h(b.input,b.ba)){Z.s(5);break}return v(Z,
a.i.i(b.input,b.ba),6);case 6:P=Z.i;case 5:if(P&&a.j)for(S=q(mq),z=S.next();!z.done;z=S.next())W=z.value,a.j[W]&&a.j[W].handleResponse(P,b);return Z.return(P)}})}
function qq(a,b,c){return w(function(d){if(a.l.xc){var e=d.return,f,g=null===(f=null===b||void 0===b?void 0:b.Ha)||void 0===f?void 0:f.sessionIndex;f=Ao({sessionIndex:g});f=Object.assign(Object.assign({},sq(c)),f);d=e.call(d,f)}else d=d.return(tq(b,c));return d})}
function tq(a,b){var c,d,e;return w(function(f){if(1==f.h){d=null===(c=null===a||void 0===a?void 0:a.Ha)||void 0===c?void 0:c.sessionIndex;var g=Ao({sessionIndex:d});if(!(g instanceof Df)){var h=new Df(Ka);Ef(h,2,g);g=h}return v(f,g,2)}e=f.i;return f.return(Promise.resolve(Object.assign(Object.assign({},sq(b)),e)))})}
function sq(a){var b={"Content-Type":"application/json"};K("enable_web_eom_visitor_data")&&B("EOM_VISITOR_DATA")?b["X-Goog-EOM-Visitor-Id"]=B("EOM_VISITOR_DATA"):B("VISITOR_DATA")&&(b["X-Goog-Visitor-Id"]=B("VISITOR_DATA"));"cors"!==a&&((a=B("INNERTUBE_CONTEXT_CLIENT_NAME"))&&(b["X-Youtube-Client-Name"]=a),(a=B("INNERTUBE_CONTEXT_CLIENT_VERSION"))&&(b["X-Youtube-Client-Version"]=a),(a=B("CHROME_CONNECTED_HEADER"))&&(b["X-Youtube-Chrome-Connected"]=a),K("forward_domain_admin_state_on_embeds")&&(a=
B("DOMAIN_ADMIN_STATE"))&&(b["X-Youtube-Domain-Admin-State"]=a));return b}
;var uq=["share/get_web_player_share_panel"],vq=["feedback"],wq=["notification/modify_channel_preference"],xq=["browse/edit_playlist"],yq=["subscription/subscribe"],zq=["subscription/unsubscribe"];function Aq(){}
r(Aq,Uo);Aq.prototype.j=function(){return yq};
Aq.prototype.h=function(a){return a.subscribeEndpoint};
Aq.prototype.i=function(a,b,c){c=void 0===c?{}:c;b.channelIds&&(a.channelIds=b.channelIds);b.siloName&&(a.siloName=b.siloName);b.params&&(a.params=b.params);c.botguardResponse&&(a.botguardResponse=c.botguardResponse);c.feature&&(a.clientFeature=c.feature)};
da.Object.defineProperties(Aq.prototype,{l:{configurable:!0,enumerable:!0,get:function(){return!0}}});function Bq(){}
r(Bq,Uo);Bq.prototype.j=function(){return zq};
Bq.prototype.h=function(a){return a.unsubscribeEndpoint};
Bq.prototype.i=function(a,b){b.channelIds&&(a.channelIds=b.channelIds);b.siloName&&(a.siloName=b.siloName);b.params&&(a.params=b.params)};
da.Object.defineProperties(Bq.prototype,{l:{configurable:!0,enumerable:!0,get:function(){return!0}}});function Cq(){}
r(Cq,Uo);Cq.prototype.j=function(){return vq};
Cq.prototype.h=function(a){return a.feedbackEndpoint};
Cq.prototype.i=function(a,b,c){a.feedbackTokens=[];b.feedbackToken&&a.feedbackTokens.push(b.feedbackToken);if(b=b.cpn||c.cpn)a.feedbackContext={cpn:b};a.isFeedbackTokenUnencrypted=!!c.is_feedback_token_unencrypted;a.shouldMerge=!1;c.extra_feedback_tokens&&(a.shouldMerge=!0,a.feedbackTokens=a.feedbackTokens.concat(c.extra_feedback_tokens))};
da.Object.defineProperties(Cq.prototype,{l:{configurable:!0,enumerable:!0,get:function(){return!0}}});function Dq(){}
r(Dq,Uo);Dq.prototype.j=function(){return wq};
Dq.prototype.h=function(a){return a.modifyChannelNotificationPreferenceEndpoint};
Dq.prototype.i=function(a,b){b.params&&(a.params=b.params);b.secondaryParams&&(a.secondaryParams=b.secondaryParams)};function Eq(){}
r(Eq,Uo);Eq.prototype.j=function(){return xq};
Eq.prototype.h=function(a){return a.playlistEditEndpoint};
Eq.prototype.i=function(a,b){b.actions&&(a.actions=b.actions);b.params&&(a.params=b.params);b.playlistId&&(a.playlistId=b.playlistId)};function Fq(){}
r(Fq,Uo);Fq.prototype.j=function(){return uq};
Fq.prototype.h=function(a){return a.webPlayerShareEntityServiceEndpoint};
Fq.prototype.i=function(a,b,c){c=void 0===c?{}:c;b.serializedShareEntity&&(a.serializedSharedEntity=b.serializedShareEntity);c.includeListId&&(a.includeListId=!0)};var Gq=new Oo;function Hq(){}
Hq.prototype.fetch=function(a,b){var c=this,d,e,f;return w(function(g){if(1==g.h){d=new window.Request(a,b);if(K("web_fetch_promise_cleanup_killswitch"))return g.return(Promise.resolve(fetch(d).then(function(h){return c.handleResponse(h)}).catch(function(h){Ln(h)})));
ta(g,3);return v(g,fetch(d),5)}if(3!=g.h)return e=g.i,g.return(c.handleResponse(e));f=wa(g);Ln(f);return g.return(void 0)})};
Hq.prototype.handleResponse=function(a){var b=a.text().then(function(c){return JSON.parse(c.replace(")]}'",""))});
a.redirected||a.ok||(b=b.then(function(c){Ln(new sk("Error: API fetch failed",a.status,a.url,c));return Object.assign(Object.assign({},c),{errorMetadata:{status:a.status}})}));
return b};
Hq[No]=[new function(a){this.key=a}(Gq)];var Iq;function Jq(a){Ul.call(this,1,arguments);this.csn=a}
r(Jq,Ul);var cm=new Vl("screen-created",Jq),Kq=[],Mq=Lq,Nq=0;function Oq(a,b,c,d,e,f,g){function h(){Ln(new sk("newScreen() parent element does not have a VE - rootVe",b))}
var k=Mq();f=new Vn({veType:b,youtubeData:f,jspbYoutubeData:void 0});e={cttAuthInfo:e,X:k};if(K("il_via_jspb")){var l=new Bg;l.Y(k);Cg(l,f.getAsJspb());c&&c.visualElement?(f=new Dg,c.clientScreenNonce&&F(f,2,c.clientScreenNonce),Eg(f,c.visualElement.getAsJspb()),g&&F(f,4,Ag[g]),G(l,5,f)):c&&h();d&&F(l,3,d);Yp(l,e,a)}else f={csn:k,pageVe:f.getAsJson()},c&&c.visualElement?(f.implicitGesture={parentCsn:c.clientScreenNonce,gesturedVe:c.visualElement.getAsJson()},g&&(f.implicitGesture.gestureType=g)):
c&&h(),d&&(f.cloneCsn=d),a?wj("screenCreated",f,a,e):pk("screenCreated",f,e);$l(cm,new Jq(k));return k}
function Pq(a,b,c,d){var e=d.filter(function(k){k.csn!==b?(k.csn=b,k=!0):k=!1;return k}),f={cttAuthInfo:eo(b),
X:b};d=q(d);for(var g=d.next();!g.done;g=d.next())g=g.value.getAsJson(),(ob(g)||!g.trackingParams&&!g.veType)&&Ln(Error("Child VE logged with no data"));if(K("il_via_jspb")){var h=new Fg;h.Y(b);Hg(h,c.getAsJspb());fb(e,function(k){k=k.getAsJspb();Uc(h,3,zg,k)});
"UNDEFINED_CSN"==b?Qq("visualElementAttached",h,f):Zp(h,f,a)}else c={csn:b,parentVe:c.getAsJson(),childVes:fb(e,function(k){return k.getAsJson()})},"UNDEFINED_CSN"==b?Qq("visualElementAttached",c,f):a?wj("visualElementAttached",c,a,f):pk("visualElementAttached",c,f)}
function Lq(){for(var a=Math.random()+"",b=[],c=0,d=0;d<a.length;d++){var e=a.charCodeAt(d);255<e&&(b[c++]=e&255,e>>=8);b[c++]=e}return zc(b,3)}
function Qq(a,b,c){Kq.push({payloadName:a,payload:b,options:c});Nq||(Nq=dm())}
function em(a){if(Kq){for(var b=q(Kq),c=b.next();!c.done;c=b.next())if(c=c.value,c.payload)if(K("il_via_jspb"))switch(c.payload.Y(a.csn),c.payloadName){case "screenCreated":Yp(c.payload,c.options);break;case "visualElementAttached":Zp(c.payload,c.options);break;case "visualElementShown":Up(c.payload,c.options);break;case "visualElementHidden":Vp(c.payload,c.options);break;case "visualElementGestured":Wp(c.payload,c.options);break;case "visualElementStateChanged":Xp(c.payload,c.options);break;default:Ln(new sk("flushQueue unable to map payloadName to JSPB setter"))}else c.payload.csn=
a.csn,wj(c.payloadName,c.payload,null,c.options);Kq.length=0}Nq=0}
;function Rq(){this.i=new Set;this.h=new Set;this.j=new Map}
Rq.prototype.clear=function(){this.i.clear();this.h.clear();this.j.clear()};
La(Rq);function Sq(){this.o=[];this.D=[];this.h=[];this.l=[];this.m=[];this.i=new Set;this.u=new Map}
function Tq(a,b,c){c=void 0===c?0:c;b.then(function(d){var e,f;a.i.has(c)&&a.j&&a.j();var g=bo(c),h=$n(c);g&&h&&((null===(e=null===d||void 0===d?void 0:d.response)||void 0===e?0:e.trackingParams)&&Pq(a.client,g,h,[Wn(d.response.trackingParams)]),(null===(f=null===d||void 0===d?void 0:d.playerResponse)||void 0===f?0:f.trackingParams)&&Pq(a.client,g,h,[Wn(d.playerResponse.trackingParams)]))})}
function Uq(a,b,c,d){d=void 0===d?0:d;if(a.i.has(d))a.o.push([b,c]);else{var e=bo(d);c=c||$n(d);e&&c&&Pq(a.client,e,c,[b])}}
Sq.prototype.clickCommand=function(a,b,c){a:{var d=a.clickTrackingParams;c=void 0===c?0:c;c=void 0===c?0:c;if(d){if(K("web_ignore_no_ve_clicks")&&(a=zn(atob(d.replace(/-/g,"+").replace(/_/g,"/"))),!a||0===a)){b=!1;break a}if(c=bo(c)){a=this.client;var e=Wn(d);var f="INTERACTION_LOGGING_GESTURE_TYPE_GENERIC_CLICK";d={cttAuthInfo:eo(c),X:c};if(K("il_via_jspb")){var g=new Ig;g.Y(c);e=e.getAsJspb();G(g,2,e);F(g,4,Ag[f]);b&&G(g,3,void 0);"UNDEFINED_CSN"==c?Qq("visualElementGestured",g,d):Wp(g,d,a)}else f=
{csn:c,ve:e.getAsJson(),gestureType:f},b&&(f.clientData=b),"UNDEFINED_CSN"==c?Qq("visualElementGestured",f,d):a?wj("visualElementGestured",f,a,d):pk("visualElementGestured",f,d);b=!0}else b=!1}else b=!1}return b};
function Vq(a,b,c){c=void 0===c?{}:c;a.i.add(c.layer||0);a.j=function(){Wq(a,b,c);var f=$n(c.layer);if(f){for(var g=q(a.o),h=g.next();!h.done;h=g.next())h=h.value,Uq(a,h[0],h[1]||f,c.layer);f=q(a.D);for(g=f.next();!g.done;g=f.next()){var k=g.value;g=void 0;g=void 0===g?0:g;h=bo(g);var l=k[0]||$n(g);if(h&&l){g=a.client;var n=k[1];k={cttAuthInfo:eo(h),X:h};K("il_via_jspb")?(n=new Kg,n.Y(h),l=l.getAsJspb(),G(n,2,l),"UNDEFINED_CSN"==h?Qq("visualElementStateChanged",n,k):Xp(n,k,g)):(l={csn:h,ve:l.getAsJson(),
clientData:n},"UNDEFINED_CSN"==h?Qq("visualElementStateChanged",l,k):g?wj("visualElementStateChanged",l,g,k):pk("visualElementStateChanged",l,k))}}}};
bo(c.layer)||a.j();if(c.sb)for(var d=q(c.sb),e=d.next();!e.done;e=d.next())Tq(a,e.value,c.layer);else Kn(Error("Delayed screen needs a data promise."))}
function Wq(a,b,c){c=void 0===c?{}:c;c.layer||(c.layer=0);var d=void 0!==c.dc?c.dc:c.layer;var e=bo(d);d=$n(d);var f;d&&(void 0!==c.parentCsn?f={clientScreenNonce:c.parentCsn,visualElement:d}:e&&"UNDEFINED_CSN"!==e&&(f={clientScreenNonce:e,visualElement:d}));var g,h=B("EVENT_ID");"UNDEFINED_CSN"===e&&h&&(g={servletData:{serializedServletEventId:h}});try{var k=Oq(a.client,b,f,c.rb,c.cttAuthInfo,g,c.zo)}catch(l){Mn(l,{Fo:b,rootVe:d,parentVisualElement:void 0,wo:e,Bo:f,rb:c.rb});Kn(l);return}fo(k,b,
c.layer,c.cttAuthInfo);if(b=e&&"UNDEFINED_CSN"!==e&&d){a:{b=q(Object.values(Un));for(f=b.next();!f.done;f=b.next())if(bo(f.value)==e){b=!0;break a}b=!1}b=!b}b&&(b=a.client,g=!0,h=(g=void 0===g?!1:g)?16:8,f={cttAuthInfo:eo(e),X:e,ub:g},K("il_via_jspb")?(h=new Jg,h.Y(e),d=d.getAsJspb(),G(h,2,d),F(h,4,g?16:8),"UNDEFINED_CSN"==e?Qq("visualElementHidden",h,f):Vp(h,f,b)):(d={csn:e,ve:d.getAsJson(),eventType:h},"UNDEFINED_CSN"==e?Qq("visualElementHidden",d,f):b?wj("visualElementHidden",d,b,f):pk("visualElementHidden",
d,f)));a.h[a.h.length-1]&&!a.h[a.h.length-1].csn&&(a.h[a.h.length-1].csn=k||"");hq({clientScreenNonce:k});Rq.getInstance().clear();d=$n(c.layer);e&&"UNDEFINED_CSN"!==e&&d&&(K("web_mark_root_visible")||K("music_web_mark_root_visible"))&&(e=k,k={cttAuthInfo:eo(e),X:e},K("il_via_jspb")?(b=new Jg,b.Y(e),f=d.getAsJspb(),G(b,2,f),F(b,4,1),"UNDEFINED_CSN"==e?Qq("visualElementShown",b,k):Up(b,k,void 0)):(b={csn:e,ve:d.getAsJson(),eventType:1},"UNDEFINED_CSN"==e?Qq("visualElementShown",b,k):pk("visualElementShown",
b,k)));a.i.delete(c.layer||0);a.j=void 0;e=q(a.u);for(k=e.next();!k.done;k=e.next())b=q(k.value),k=b.next().value,b=b.next().value,b.has(c.layer)&&d&&Uq(a,k,d,c.layer);for(c=0;c<a.l.length;c++){e=a.l[c];try{e()}catch(l){Kn(l)}}for(c=a.l.length=0;c<a.m.length;c++){e=a.m[c];try{e()}catch(l){Kn(l)}}}
;function Xq(){var a;return w(function(b){if(1==b.h)return v(b,pq(),2);if(a=b.i)return a.errorMetadata?(Kn(Error("Datasync IDs fetch responded with "+a.errorMetadata.code+": "+a.error)),b.return(void 0)):b.return(a.xo);Ln(Error("Network request to get Datasync IDs failed."));return b.return(void 0)})}
;var Yq=x.caches,Zq;function $q(a){var b=a.indexOf(":");return-1===b?{Cb:a}:{Cb:a.substring(0,b),datasyncId:a.substring(b+1)}}
function ar(){return w(function(a){if(void 0!==Zq)return a.return(Zq);Zq=new Promise(function(b){var c;return w(function(d){switch(d.h){case 1:return ta(d,2),v(d,Yq.open("test-only"),4);case 4:return v(d,Yq.delete("test-only"),5);case 5:va(d,3);break;case 2:if(c=wa(d),c instanceof Error&&"SecurityError"===c.name)return b(!1),d.return();case 3:b("caches"in window),d.h=0}})});
return a.return(Zq)})}
function br(a){var b,c,d,e,f,g,h;w(function(k){if(1==k.h)return v(k,ar(),2);if(3!=k.h){if(!k.i)return k.return(!1);b=[];return v(k,Yq.keys(),3)}c=k.i;d=q(c);for(e=d.next();!e.done;e=d.next())f=e.value,g=$q(f),h=g.datasyncId,!h||a.includes(h)||b.push(Yq.delete(f));return k.return(Promise.all(b).then(function(l){return l.some(function(n){return n})}))})}
;function cr(){Xq().then(function(a){if(a&&(Dl(a),br(a),K("clear_user_partitioned_ls"))){var b=void 0===b?{}:b;"_start"in kp("cupls")&&Y("aa",void 0,"cupls");var c=Ap();c.cupls&&delete c.cupls;var d={timerName:"cupls",info:{},tick:{},span:{},jspbInfo:[]};yp().push(d);c.cupls=d;qp("cupls");ip();pp("cupls").useGel=!0;zp("cupls").info.actionType="cupls";b.cttAuthInfo&&(pp("cupls").cttAuthInfo=b.cttAuthInfo);oh("cuplsTIMING_ACTION","cupls");Y("_start",b.startTime,"cupls");b={actionType:Hp("cupls")};(c=
bo())&&"UNDEFINED_CSN"!==c&&(b.clientScreenNonce=c);hq(b,"cupls");op("cupls");Y("cuplss",void 0,"cupls");try{try{var e=!!self.localStorage}catch(t){e=!1}if(e)for(var f=Object.keys(window.localStorage),g=q(f),h=g.next();!h.done;h=g.next()){var k=h.value;var l=k.match(/(.*)::.*::.*/);var n=null!==l?l[1]:void 0;e=n;void 0===e||a.includes(e)||self.localStorage.removeItem(k)}Y("cuplsc",void 0,"cupls")}catch(t){Kn(t),Y("cuplse",void 0,"cupls")}}})}
function dr(){var a=new Om;Vh.N(function(){a.H()?cr():a.i.add("publicytnetworkstatus-online",cr,!0,void 0,void 0)})}
;function er(a){a&&(a.dataset?a.dataset[fr("loaded")]="true":a.setAttribute("data-loaded","true"))}
function gr(a,b){return a?a.dataset?a.dataset[fr(b)]:a.getAttribute("data-"+b):null}
var hr={};function fr(a){return hr[a]||(hr[a]=String(a).replace(/\-([a-z])/g,function(b,c){return c.toUpperCase()}))}
;var ir=/\.vflset|-vfl[a-zA-Z0-9_+=-]+/,jr=/-[a-zA-Z]{2,3}_[a-zA-Z]{2,3}(?=(\/|$))/;function kr(a,b,c){c=void 0===c?null:c;if(window.spf&&spf.script){c="";if(a){var d=a.indexOf("jsbin/"),e=a.lastIndexOf(".js"),f=d+6;-1<d&&-1<e&&e>f&&(c=a.substring(f,e),c=c.replace(ir,""),c=c.replace(jr,""),c=c.replace("debug-",""),c=c.replace("tracing-",""))}spf.script.load(a,c,b)}else lr(a,b,c)}
function lr(a,b,c){c=void 0===c?null:c;var d=mr(a),e=document.getElementById(d),f=e&&gr(e,"loaded"),g=e&&!f;f?b&&b():(b&&(f=Ji(d,b),b=""+Pa(b),nr[b]=f),g||(e=or(a,d,function(){gr(e,"loaded")||(er(e),Mi(d),Lh(Va(Ni,d),0))},c)))}
function or(a,b,c,d){d=void 0===d?null:d;var e=vd("SCRIPT");e.id=b;e.onload=function(){c&&setTimeout(c,0)};
e.onreadystatechange=function(){switch(e.readyState){case "loaded":case "complete":e.onload()}};
d&&e.setAttribute("nonce",d);rd(e,uf(a));a=document.getElementsByTagName("head")[0]||document.body;a.insertBefore(e,a.firstChild);return e}
function pr(a){a=mr(a);var b=document.getElementById(a);b&&(Ni(a),b.parentNode.removeChild(b))}
function qr(a,b){a&&b&&(a=""+Pa(b),(a=nr[a])&&Li(a))}
function mr(a){var b=document.createElement("a");Qb(b,a);a=b.href.replace(/^[a-zA-Z]+:\/\//,"//");return"js-"+Vb(a)}
var nr={};var rr=[],sr=!1;function tr(){if(!K("disable_biscotti_fetch_for_ad_blocker_detection")&&!K("disable_biscotti_fetch_entirely_for_all_web_clients")&&qo()&&"1"!=qb()){var a=function(){sr=!0;"google_ad_status"in window?oh("DCLKSTAT",1):oh("DCLKSTAT",2)};
try{kr("//static.doubleclick.net/instream/ad_status.js",a)}catch(b){}rr.push(Vh.N(function(){if(!(sr||"google_ad_status"in window)){try{qr("//static.doubleclick.net/instream/ad_status.js",a)}catch(b){}sr=!0;oh("DCLKSTAT",3)}},5E3))}}
function ur(){var a=Number(B("DCLKSTAT",0));return isNaN(a)?0:a}
;function vr(){this.state=1;this.h=null}
m=vr.prototype;
m.initialize=function(a,b,c){var d,e;if(a.program){var f=null!==(d=a.interpreterScript)&&void 0!==d?d:null,g=null!==(e=a.interpreterUrl)&&void 0!==e?e:null;if(a.interpreterSafeScript){f=a.interpreterSafeScript;Ab("From proto message. b/166824318");f=f.privateDoNotAccessOrElseSafeScriptWrappedValue||"";var h=xb();f=h?h.createScript(f):f;f=(new Cb(f)).toString()}a.interpreterSafeUrl&&(g=a.interpreterSafeUrl,Ab("From proto message. b/166824318"),g=Gb(g.privateDoNotAccessOrElseTrustedResourceUrlWrappedValue||"").toString());
wr(this,f,g,a.program,b,c)}else Ln(Error("Cannot initialize botguard without program"))};
function wr(a,b,c,d,e,f){var g=void 0===g?"trayride":g;c?(a.state=2,kr(c,function(){window[g]?xr(a,d,g,e):(a.state=3,pr(c),Ln(new sk("Unable to load Botguard","from "+c)))},f)):b?(f=vd("SCRIPT"),f.textContent=b,f.nonce=Tb(),document.head.appendChild(f),document.head.removeChild(f),window[g]?xr(a,d,g,e):(a.state=4,Ln(new sk("Unable to load Botguard from JS")))):Ln(new sk("Unable to load VM; no url or JS provided"))}
m.isInitialized=function(){return!!this.h};
m.getState=function(){return this.state};
function xr(a,b,c,d){a.state=5;try{var e=new kd({program:b,globalName:c});e.uc.then(function(){a.state=6;d&&d(b)});
yr(a,e)}catch(f){a.state=7,f instanceof Error&&Ln(f)}}
m.invoke=function(a){a=void 0===a?{}:a;if(this.h){var b=this.h;a={qb:a};if(b.i)throw Error("Already disposed");b=b.l([a.qb,a.wc])}else b=null;return b};
m.dispose=function(){yr(this,null);this.state=8};
function yr(a,b){Wd(a.h);a.h=b}
;var zr=new vr;function Ar(){return zr.isInitialized()}
function Br(a){a=void 0===a?{}:a;return zr.invoke(a)}
;function Cr(a){var b=this;var c=void 0===c?0:c;var d=void 0===d?Uh():d;this.l=c;this.j=d;this.i=new jd;this.h=a;a={};c=q(this.h.entries());for(d=c.next();!d.done;a={ta:a.ta,Ca:a.Ca},d=c.next()){var e=q(d.value);d=e.next().value;e=e.next().value;a.Ca=d;a.ta=e;d=function(f){return function(){f.ta.fb();b.h[f.Ca].Sa=!0;b.h.every(function(g){return!0===g.Sa})&&b.i.resolve()}}(a);
e=Qh(d,Dr(this,a.ta));this.h[a.Ca]=Object.assign(Object.assign({},a.ta),{fb:d,Na:e})}}
function Er(a){var b=Array.from(a.h.keys()).sort(function(d,e){return Dr(a,a.h[e])-Dr(a,a.h[d])});
b=q(b);for(var c=b.next();!c.done;c=b.next())c=a.h[c.value],void 0===c.Na||c.Sa||(a.j.U(c.Na),Qh(c.fb,10))}
Cr.prototype.cancel=function(){for(var a=q(this.h),b=a.next();!b.done;b=a.next())b=b.value,void 0===b.Na||b.Sa||this.j.U(b.Na),b.Sa=!0;this.i.resolve()};
function Dr(a,b){var c;return null!==(c=b.priority)&&void 0!==c?c:a.l}
;function Fr(a){this.state=a;this.plugins=[];this.m=void 0}
Fr.prototype.install=function(){this.plugins.push.apply(this.plugins,ha(Fa.apply(0,arguments)))};
Fr.prototype.transition=function(a,b){var c=this,d=this.D.find(function(f){return f.from===c.state&&f.B===a});
if(d){this.j&&(Er(this.j),this.j=void 0);this.state=a;d=d.action.bind(this);var e=this.plugins.filter(function(f){return f[a]}).map(function(f){return f[a]});
d(Gr(this,e,this.m),b)}else throw Error("no transition specified from "+this.state+" to "+a);};
function Gr(a,b,c){return function(){var d=Fa.apply(0,arguments),e=b.filter(function(k){var l;return 10===(null!==(l=null!==c&&void 0!==c?c:k.priority)&&void 0!==l?l:0)}),f=b.filter(function(k){var l;
return 10!==(null!==(l=null!==c&&void 0!==c?c:k.priority)&&void 0!==l?l:0)});
Uh();var g={};e=q(e);for(var h=e.next();!h.done;g={Da:g.Da},h=e.next())g.Da=h.value,Sh(function(k){return function(){k.Da.la.apply(k.Da,ha(d))}}(g));
f=f.map(function(k){var l;return{fb:function(){k.la.apply(k,ha(d))},
priority:null!==(l=null!==c&&void 0!==c?c:k.priority)&&void 0!==l?l:0}});
f.length&&(a.j=new Cr(f))}}
da.Object.defineProperties(Fr.prototype,{currentState:{configurable:!0,enumerable:!0,get:function(){return this.state}}});function Hr(a){Fr.call(this,void 0===a?"document_active":a);var b=this;this.m=10;this.h=new Map;this.D=[{from:"document_active",B:"document_disposed_preventable",action:this.u},{from:"document_active",B:"document_disposed",action:this.l},{from:"document_disposed_preventable",B:"document_disposed",action:this.l},{from:"document_disposed_preventable",B:"flush_logs",action:this.o},{from:"document_disposed_preventable",B:"document_active",action:this.i},{from:"document_disposed",B:"flush_logs",action:this.o},
{from:"document_disposed",B:"document_active",action:this.i},{from:"document_disposed",B:"document_disposed",action:function(){}},
{from:"flush_logs",B:"document_active",action:this.i}];window.addEventListener("pagehide",function(c){b.transition("document_disposed",c)});
window.addEventListener("beforeunload",function(c){b.transition("document_disposed_preventable",c)})}
r(Hr,Fr);Hr.prototype.u=function(a,b){if(!this.h.get("document_disposed_preventable")&&(a(b),(null===b||void 0===b?0:b.defaultPrevented)||(null===b||void 0===b?0:b.returnValue))){b.returnValue||(b.returnValue=!0);b.defaultPrevented||b.preventDefault();this.h=new Map;this.transition("document_active");return}this.h.set("document_disposed_preventable",!0);this.h.get("document_disposed")?this.transition("flush_logs"):this.transition("document_disposed")};
Hr.prototype.l=function(a,b){this.h.get("document_disposed")?this.transition("document_active"):(a(b),this.h.set("document_disposed",!0),this.transition("flush_logs"))};
Hr.prototype.o=function(a,b){a(b);this.transition("document_active")};
Hr.prototype.i=function(){this.h=new Map};function Ir(a){Fr.call(this,void 0===a?"document_visibility_unknown":a);var b=this;this.D=[{from:"document_visibility_unknown",B:"document_visible",action:this.i},{from:"document_visibility_unknown",B:"document_hidden",action:this.h},{from:"document_visibility_unknown",B:"document_foregrounded",action:this.o},{from:"document_visibility_unknown",B:"document_backgrounded",action:this.l},{from:"document_visible",B:"document_hidden",action:this.h},{from:"document_visible",B:"document_foregrounded",action:this.o},
{from:"document_visible",B:"document_visible",action:this.i},{from:"document_foregrounded",B:"document_visible",action:this.i},{from:"document_foregrounded",B:"document_hidden",action:this.h},{from:"document_foregrounded",B:"document_foregrounded",action:this.o},{from:"document_hidden",B:"document_visible",action:this.i},{from:"document_hidden",B:"document_backgrounded",action:this.l},{from:"document_hidden",B:"document_hidden",action:this.h},{from:"document_backgrounded",B:"document_hidden",action:this.h},
{from:"document_backgrounded",B:"document_backgrounded",action:this.l},{from:"document_backgrounded",B:"document_visible",action:this.i}];document.addEventListener("visibilitychange",function(c){"visible"===document.visibilityState?b.transition("document_visible",c):b.transition("document_hidden",c)});
K("visibility_lifecycles_dynamic_backgrounding")&&(window.addEventListener("blur",function(c){b.transition("document_backgrounded",c)}),window.addEventListener("focus",function(c){b.transition("document_foregrounded",c)}))}
r(Ir,Fr);Ir.prototype.i=function(a,b){a(b);K("visibility_lifecycles_dynamic_backgrounding")&&this.transition("document_foregrounded")};
Ir.prototype.h=function(a,b){a(b);K("visibility_lifecycles_dynamic_backgrounding")&&this.transition("document_backgrounded")};
Ir.prototype.l=function(a,b){a(b)};
Ir.prototype.o=function(a,b){a(b)};function Jr(){this.h=new Hr;this.i=new Ir}
Jr.prototype.install=function(){var a=Fa.apply(0,arguments);this.h.install.apply(this.h,ha(a));this.i.install.apply(this.i,ha(a))};function Kr(){Jr.call(this);var a={};this.install((a.document_disposed={la:this.j},a));a={};this.install((a.flush_logs={la:this.l},a))}
var Lr;r(Kr,Jr);Kr.prototype.l=function(){pk("finalPayload",{csn:bo()})};
Kr.prototype.j=function(){On(Pn)};function Mr(){}
Mr.getInstance=function(){var a=A("ytglobal.storage_");a||(a=new Mr,y("ytglobal.storage_",a,void 0));return a};
Mr.prototype.estimate=function(){var a,b,c;return w(function(d){c=navigator;return(null===(a=c.storage)||void 0===a?0:a.estimate)?d.return(c.storage.estimate()):(null===(b=c.webkitTemporaryStorage)||void 0===b?0:b.queryUsageAndQuota)?d.return(Nr()):d.return()})};
function Nr(){var a=navigator;return new Promise(function(b,c){var d;null!==(d=a.webkitTemporaryStorage)&&void 0!==d&&d.queryUsageAndQuota?a.webkitTemporaryStorage.queryUsageAndQuota(function(e,f){b({usage:e,quota:f})},function(e){c(e)}):c(Error("webkitTemporaryStorage is not supported."))})}
y("ytglobal.storageClass_",Mr,void 0);function nk(a,b){var c=this;this.handleError=a;this.h=b;this.i=!1;void 0===self.document||self.addEventListener("beforeunload",function(){c.i=!0});
this.j=Math.random()<=rh("ytidb_transaction_ended_event_rate_limit",.02)}
nk.prototype.logEvent=function(a,b){switch(a){case "IDB_DATA_CORRUPTED":K("idb_data_corrupted_killswitch")||this.h("idbDataCorrupted",b);break;case "IDB_UNEXPECTEDLY_CLOSED":this.h("idbUnexpectedlyClosed",b);break;case "IS_SUPPORTED_COMPLETED":K("idb_is_supported_completed_killswitch")||this.h("idbIsSupportedCompleted",b);break;case "QUOTA_EXCEEDED":Or(this,b);break;case "TRANSACTION_ENDED":this.j&&this.h("idbTransactionEnded",b);break;case "TRANSACTION_UNEXPECTEDLY_ABORTED":a=Object.assign(Object.assign({},
b),{hasWindowUnloaded:this.i}),this.h("idbTransactionAborted",a)}};
function Or(a,b){Mr.getInstance().estimate().then(function(c){c=Object.assign(Object.assign({},b),{isSw:void 0===self.document,isIframe:self!==self.top,deviceStorageUsageMbytes:Pr(null===c||void 0===c?void 0:c.usage),deviceStorageQuotaMbytes:Pr(null===c||void 0===c?void 0:c.quota)});a.h("idbQuotaExceeded",c)})}
function Pr(a){return"undefined"===typeof a?"-1":String(Math.ceil(a/1048576))}
;var Qr=window;
function Rr(){var a=Qr.uaChPolyfill.state;if(0===a.type)pk("clientHintsPolyfillEvent",{clientHintsSupported:!1});else{var b=navigator.userAgent,c=void 0!==a.syntheticUa&&a.syntheticUa===b,d={clientHintsSupported:!0,uaAccessedBeforePolyfill:a.didAccessUaBeforePolyfillAvailable,syntheticUaMatches:c};a.didAccessUaBeforePolyfillAvailable&&(d.uaAccessBeforePolyfillCount=a.uaAccessBeforePolyfillCount,a.firstAccessUaError&&(d.firstUaAccessStack=String(a.firstAccessUaError.stack).replace(/\n/g,""),Kn(a.firstAccessUaError)),
d.polyfillAvailabilityDelayMs=a.polyfillAvailabilityDelay);pk("clientHintsPolyfillEvent",d);c||(b={syntheticUa:a.syntheticUa,ua:b},b.brand=a.data.brands.map(function(e){return'"'+e.brand+'"; v="'+e.version+'"'}),b.mobileness=a.data.mobile,a=a.data.values,a.architecture&&(b.platformArchitecture=a.architecture),a.model&&(b.model=a.model),a.platform&&(b.platformBrand=a.platform),a.platformVersion&&(b.platformVersion=a.platformVersion),a.uaFullVersion&&(b.fullVersion=a.uaFullVersion),pk("clientHintsPolyfillDiagnostics",
b))}}
var Sr=!1;function Tr(){var a;1===(null===(a=Qr.uaChPolyfill)||void 0===a?void 0:a.state.type)?Sr||(Qr.uaChPolyfill.onReady=Tr,Sr=!0):Qr.uaChPolyfill&&Rr()}
;function Ur(a,b,c){I.call(this);var d=this;c=c||B("POST_MESSAGE_ORIGIN",void 0)||window.document.location.protocol+"//"+window.document.location.hostname;this.j=b||null;this.L="*";this.l=c;this.sessionId=null;this.channel="widget";this.M=!!a;this.A=function(e){a:if(!("*"!=d.l&&e.origin!=d.l||d.j&&e.source!=d.j||"string"!==typeof e.data)){try{var f=JSON.parse(e.data)}catch(g){break a}if(!(null==f||d.M&&(d.sessionId&&d.sessionId!=f.id||d.channel&&d.channel!=f.channel))&&f)switch(f.event){case "listening":"null"!=
e.origin&&(d.l=d.L=e.origin);d.j=e.source;d.sessionId=f.id;d.i&&(d.i(),d.i=null);break;case "command":d.m&&(!d.u||0<=cb(d.u,f.func))&&d.m(f.func,f.args,e.origin)}}};
this.u=this.i=this.m=null;window.addEventListener("message",this.A)}
r(Ur,I);Ur.prototype.sendMessage=function(a,b){if(b=b||this.j){this.sessionId&&(a.id=this.sessionId);this.channel&&(a.channel=this.channel);try{var c=JSON.stringify(a);b.postMessage(c,this.L)}catch(d){Bh(d)}}};
Ur.prototype.I=function(){window.removeEventListener("message",this.A);I.prototype.I.call(this)};function Vr(){this.i=[];this.isReady=!1;this.j={};var a=this.h=new Ur(!!B("WIDGET_ID_ENFORCE")),b=this.fc.bind(this);a.m=b;a.u=null;this.h.channel="widget";if(a=B("WIDGET_ID"))this.h.sessionId=a}
m=Vr.prototype;m.fc=function(a,b,c){"addEventListener"===a&&b?(a=b[0],this.j[a]||"onReady"===a||(this.addEventListener(a,Wr(this,a)),this.j[a]=!0)):this.lb(a,b,c)};
m.lb=function(){};
function Wr(a,b){return function(c){return a.sendMessage(b,c)}}
m.addEventListener=function(){};
m.Ub=function(){this.isReady=!0;this.sendMessage("initialDelivery",this.Ya());this.sendMessage("onReady");db(this.i,this.Ib,this);this.i=[]};
m.Ya=function(){return null};
function Xr(a,b){a.sendMessage("infoDelivery",b)}
m.Ib=function(a){this.isReady?this.h.sendMessage(a):this.i.push(a)};
m.sendMessage=function(a,b){this.Ib({event:a,info:void 0===b?null:b})};
m.dispose=function(){this.h=null};function Yr(a){return(0===a.search("cue")||0===a.search("load"))&&"loadModule"!==a}
function Zr(a,b,c){if("string"===typeof a)return{videoId:a,startSeconds:b,suggestedQuality:c};b=["endSeconds","startSeconds","mediaContentUrl","suggestedQuality","videoId"];c={};for(var d=0;d<b.length;d++){var e=b[d];a[e]&&(c[e]=a[e])}return c}
function $r(a,b,c,d){if(Oa(a)&&!Array.isArray(a)){b="playlist list listType index startSeconds suggestedQuality".split(" ");c={};for(d=0;d<b.length;d++){var e=b[d];a[e]&&(c[e]=a[e])}return c}b={index:b,startSeconds:c,suggestedQuality:d};"string"===typeof a&&16===a.length?b.list="PL"+a:b.playlist=a;return b}
;function as(a){Vr.call(this);this.listeners=[];this.api=a;this.addEventListener("onReady",this.onReady.bind(this));this.addEventListener("onVideoProgress",this.qc.bind(this));this.addEventListener("onVolumeChange",this.sc.bind(this));this.addEventListener("onApiChange",this.kc.bind(this));this.addEventListener("onPlaybackQualityChange",this.nc.bind(this));this.addEventListener("onPlaybackRateChange",this.oc.bind(this));this.addEventListener("onStateChange",this.pc.bind(this));this.addEventListener("onWebglSettingsChanged",
this.tc.bind(this))}
r(as,Vr);m=as.prototype;
m.lb=function(a,b,c){if(this.api.isExternalMethodAvailable(a,c)){b=b||[];if(0<b.length&&Yr(a)){var d=b;if(Oa(d[0])&&!Array.isArray(d[0]))var e=d[0];else switch(e={},a){case "loadVideoById":case "cueVideoById":e=Zr(d[0],void 0!==d[1]?Number(d[1]):void 0,d[2]);break;case "loadVideoByUrl":case "cueVideoByUrl":e=d[0];"string"===typeof e&&(e={mediaContentUrl:e,startSeconds:void 0!==d[1]?Number(d[1]):void 0,suggestedQuality:d[2]});b:{if((d=e.mediaContentUrl)&&(d=/\/([ve]|embed)\/([^#?]+)/.exec(d))&&d[2]){d=
d[2];break b}d=null}e.videoId=d;e=Zr(e);break;case "loadPlaylist":case "cuePlaylist":e=$r(d[0],d[1],d[2],d[3])}b.length=1;b[0]=e}this.api.handleExternalCall(a,b,c);Yr(a)&&Xr(this,this.Ya())}};
m.onReady=function(){var a=this.Ub.bind(this);this.h.i=a};
m.addEventListener=function(a,b){this.listeners.push({eventType:a,listener:b});this.api.addEventListener(a,b)};
m.Ya=function(){if(!this.api)return null;var a=this.api.getApiInterface();ib(a,"getVideoData");for(var b={apiInterface:a},c=0,d=a.length;c<d;c++){var e=a[c];if(0===e.search("get")||0===e.search("is")){var f=0;0===e.search("get")?f=3:0===e.search("is")&&(f=2);f=e.charAt(f).toLowerCase()+e.substr(f+1);try{var g=this.api[e]();b[f]=g}catch(h){}}}b.videoData=this.api.getVideoData();b.currentTimeLastUpdated_=Date.now()/1E3;return b};
m.pc=function(a){a={playerState:a,currentTime:this.api.getCurrentTime(),duration:this.api.getDuration(),videoData:this.api.getVideoData(),videoStartBytes:0,videoBytesTotal:this.api.getVideoBytesTotal(),videoLoadedFraction:this.api.getVideoLoadedFraction(),playbackQuality:this.api.getPlaybackQuality(),availableQualityLevels:this.api.getAvailableQualityLevels(),currentTimeLastUpdated_:Date.now()/1E3,playbackRate:this.api.getPlaybackRate(),mediaReferenceTime:this.api.getMediaReferenceTime()};this.api.getVideoUrl&&
(a.videoUrl=this.api.getVideoUrl());this.api.getVideoContentRect&&(a.videoContentRect=this.api.getVideoContentRect());this.api.getProgressState&&(a.progressState=this.api.getProgressState());this.api.getPlaylist&&(a.playlist=this.api.getPlaylist());this.api.getPlaylistIndex&&(a.playlistIndex=this.api.getPlaylistIndex());this.api.getStoryboardFormat&&(a.storyboardFormat=this.api.getStoryboardFormat());Xr(this,a)};
m.nc=function(a){Xr(this,{playbackQuality:a})};
m.oc=function(a){Xr(this,{playbackRate:a})};
m.kc=function(){for(var a=this.api.getOptions(),b={namespaces:a},c=0,d=a.length;c<d;c++){var e=a[c],f=this.api.getOptions(e);b[e]={options:f};for(var g=0,h=f.length;g<h;g++){var k=f[g],l=this.api.getOption(e,k);b[e][k]=l}}this.sendMessage("apiInfoDelivery",b)};
m.sc=function(){Xr(this,{muted:this.api.isMuted(),volume:this.api.getVolume()})};
m.qc=function(a){a={currentTime:a,videoBytesLoaded:this.api.getVideoBytesLoaded(),videoLoadedFraction:this.api.getVideoLoadedFraction(),currentTimeLastUpdated_:Date.now()/1E3,playbackRate:this.api.getPlaybackRate(),mediaReferenceTime:this.api.getMediaReferenceTime()};this.api.getProgressState&&(a.progressState=this.api.getProgressState());Xr(this,a)};
m.tc=function(){var a={sphericalProperties:this.api.getSphericalProperties()};Xr(this,a)};
m.dispose=function(){Vr.prototype.dispose.call(this);for(var a=0;a<this.listeners.length;a++){var b=this.listeners[a];this.api.removeEventListener(b.eventType,b.listener)}this.listeners=[]};function bs(a){I.call(this);this.i={};this.started=!1;this.connection=a;this.connection.subscribe("command",this.Eb,this)}
r(bs,I);m=bs.prototype;m.start=function(){this.started||this.h()||(this.started=!0,this.connection.ma("RECEIVING"))};
m.ma=function(a,b){this.started&&!this.h()&&this.connection.ma(a,b)};
m.Eb=function(a,b,c){if(this.started&&!this.h()){var d=b||{};switch(a){case "addEventListener":"string"===typeof d.event&&this.addListener(d.event);break;case "removeEventListener":"string"===typeof d.event&&this.removeListener(d.event);break;default:this.api.isReady()&&this.api.isExternalMethodAvailable(a,c||null)&&(b=cs(a,b||{}),c=this.api.handleExternalCall(a,b,c||null),(c=ds(a,c))&&this.ma(a,c))}}};
m.addListener=function(a){if(!(a in this.i)){var b=this.lc.bind(this,a);this.i[a]=b;this.addEventListener(a,b)}};
m.lc=function(a,b){this.started&&!this.h()&&this.connection.ma(a,this.Xa(a,b))};
m.Xa=function(a,b){if(null!=b)return{value:b}};
m.removeListener=function(a){a in this.i&&(this.removeEventListener(a,this.i[a]),delete this.i[a])};
m.I=function(){var a=this.connection;a.h()||Vf(a.i,"command",this.Eb,this);this.connection=null;for(var b in this.i)this.i.hasOwnProperty(b)&&this.removeListener(b);I.prototype.I.call(this)};function es(a,b){bs.call(this,b);this.api=a;this.start()}
r(es,bs);es.prototype.addEventListener=function(a,b){this.api.addEventListener(a,b)};
es.prototype.removeEventListener=function(a,b){this.api.removeEventListener(a,b)};
function cs(a,b){switch(a){case "loadVideoById":return a=Zr(b),[a];case "cueVideoById":return a=Zr(b),[a];case "loadVideoByPlayerVars":return[b];case "cueVideoByPlayerVars":return[b];case "loadPlaylist":return a=$r(b),[a];case "cuePlaylist":return a=$r(b),[a];case "seekTo":return[b.seconds,b.allowSeekAhead];case "playVideoAt":return[b.index];case "setVolume":return[b.volume];case "setPlaybackQuality":return[b.suggestedQuality];case "setPlaybackRate":return[b.suggestedRate];case "setLoop":return[b.loopPlaylists];
case "setShuffle":return[b.shufflePlaylist];case "getOptions":return[b.module];case "getOption":return[b.module,b.option];case "setOption":return[b.module,b.option,b.value];case "handleGlobalKeyDown":return[b.keyCode,b.shiftKey,b.ctrlKey,b.altKey,b.metaKey,b.key,b.code]}return[]}
function ds(a,b){switch(a){case "isMuted":return{muted:b};case "getVolume":return{volume:b};case "getPlaybackRate":return{playbackRate:b};case "getAvailablePlaybackRates":return{availablePlaybackRates:b};case "getVideoLoadedFraction":return{videoLoadedFraction:b};case "getPlayerState":return{playerState:b};case "getCurrentTime":return{currentTime:b};case "getPlaybackQuality":return{playbackQuality:b};case "getAvailableQualityLevels":return{availableQualityLevels:b};case "getDuration":return{duration:b};
case "getVideoUrl":return{videoUrl:b};case "getVideoEmbedCode":return{videoEmbedCode:b};case "getPlaylist":return{playlist:b};case "getPlaylistIndex":return{playlistIndex:b};case "getOptions":return{options:b};case "getOption":return{option:b}}}
es.prototype.Xa=function(a,b){switch(a){case "onReady":return;case "onStateChange":return{playerState:b};case "onPlaybackQualityChange":return{playbackQuality:b};case "onPlaybackRateChange":return{playbackRate:b};case "onError":return{errorCode:b}}return bs.prototype.Xa.call(this,a,b)};
es.prototype.I=function(){bs.prototype.I.call(this);delete this.api};function fs(a){a=void 0===a?!1:a;I.call(this);this.i=new J(a);Yd(this,Va(Wd,this.i))}
Xa(fs,I);fs.prototype.subscribe=function(a,b,c){return this.h()?0:this.i.subscribe(a,b,c)};
fs.prototype.l=function(a,b){this.h()||this.i.ka.apply(this.i,arguments)};function gs(a,b,c){fs.call(this);this.j=a;this.destination=b;this.id=c}
r(gs,fs);gs.prototype.ma=function(a,b){this.h()||this.j.ma(this.destination,this.id,a,b)};
gs.prototype.I=function(){this.destination=this.j=null;fs.prototype.I.call(this)};function hs(a,b,c){I.call(this);this.destination=a;this.origin=c;this.i=Ih(window,"message",this.j.bind(this));this.connection=new gs(this,a,b);Yd(this,Va(Wd,this.connection))}
r(hs,I);hs.prototype.ma=function(a,b,c,d){this.h()||a!==this.destination||(a={id:b,command:c},d&&(a.data=d),this.destination.postMessage(wf(a),this.origin))};
hs.prototype.j=function(a){var b;if(b=!this.h())if(b=a.origin===this.origin)a:{b=this.destination;do{b:{var c=a.source;do{if(c===b){c=!0;break b}if(c===c.parent)break;c=c.parent}while(null!=c);c=!1}if(c){b=!0;break a}b=b.opener}while(null!=b);b=!1}if(b&&(b=a.data,"string"===typeof b)){try{b=JSON.parse(b)}catch(d){return}b.command&&(c=this.connection,c.h()||c.l("command",b.command,b.data,a.origin))}};
hs.prototype.I=function(){Jh(this.i);this.destination=null;I.prototype.I.call(this)};function is(a){a=a||{};var b={},c={};this.url=a.url||"";this.args=a.args||sb(b);this.assets=a.assets||{};this.attrs=a.attrs||sb(c);this.fallback=a.fallback||null;this.fallbackMessage=a.fallbackMessage||null;this.html5=!!a.html5;this.disable=a.disable||{};this.loaded=!!a.loaded;this.messages=a.messages||{}}
is.prototype.clone=function(){var a=new is,b;for(b in this)if(this.hasOwnProperty(b)){var c=this[b];"object"==Ma(c)?a[b]=sb(c):a[b]=c}return a};var js=/cssbin\/(?:debug-)?([a-zA-Z0-9_-]+?)(?:-2x|-web|-rtl|-vfl|.css)/;function ks(a){a=a||"";if(window.spf){var b=a.match(js);spf.style.load(a,b?b[1]:"",void 0)}else ls(a)}
function ls(a){var b=ms(a),c=document.getElementById(b),d=c&&gr(c,"loaded");d||c&&!d||(c=ns(a,b,function(){gr(c,"loaded")||(er(c),Mi(b),Lh(Va(Ni,b),0))}))}
function ns(a,b,c){var d=document.createElement("link");d.id=b;d.onload=function(){c&&setTimeout(c,0)};
a=uf(a);Rb(d,a);(document.getElementsByTagName("head")[0]||document.body).appendChild(d);return d}
function ms(a){var b=vd("A");Ab("This URL is never added to the DOM");Qb(b,new Ib(a,Jb));a=b.href.replace(/^[a-zA-Z]+:\/\//,"//");return"css-"+Vb(a)}
;function os(){I.call(this);this.i=[]}
r(os,I);os.prototype.I=function(){for(;this.i.length;){var a=this.i.pop();a.target.removeEventListener(a.name,a.la,void 0)}I.prototype.I.call(this)};function ps(){os.apply(this,arguments)}
r(ps,os);function qs(a,b,c,d){I.call(this);var e=this;this.M=b;this.webPlayerContextConfig=d;this.Ua=!1;this.api={};this.Ea=this.u=null;this.S=new J;this.i={};this.ga=this.Fa=this.elementId=this.Va=this.config=null;this.Z=!1;this.l=this.A=null;this.Ga={};this.Lb=["onReady"];this.lastError=null;this.mb=NaN;this.L={};this.Mb=new ps(this);this.oa=0;this.j=this.m=a;Yd(this,Va(Wd,this.S));rs(this);ss(this);Yd(this,Va(Wd,this.Mb));c?this.oa=Lh(function(){e.loadNewVideoConfig(c)},0):d&&(ts(this),us(this))}
r(qs,I);m=qs.prototype;m.getId=function(){return this.M};
m.loadNewVideoConfig=function(a){if(!this.h()){this.oa&&(Mh(this.oa),this.oa=0);var b=a||{};b instanceof is||(b=new is(b));this.config=b;this.setConfig(a);us(this);this.isReady()&&vs(this)}};
function ts(a){var b,c;a.webPlayerContextConfig?c=a.webPlayerContextConfig.rootElementId:c=a.config.attrs.id;a.elementId=c||a.elementId;"video-player"===a.elementId&&(a.elementId=a.M,a.webPlayerContextConfig?a.webPlayerContextConfig.rootElementId=a.M:a.config.attrs.id=a.M);(null===(b=a.j)||void 0===b?void 0:b.id)===a.elementId&&(a.elementId+="-player",a.webPlayerContextConfig?a.webPlayerContextConfig.rootElementId=a.elementId:a.config.attrs.id=a.elementId)}
m.setConfig=function(a){var b;this.Va=a;this.config=ws(a);ts(this);this.Fa||(this.Fa=xs(this,(null===(b=this.config.args)||void 0===b?void 0:b.jsapicallback)||"onYouTubePlayerReady"));this.config.args?this.config.args.jsapicallback=null:this.config.args={jsapicallback:null};var c;if(null===(c=this.config)||void 0===c?0:c.attrs)a=this.config.attrs,(c=a.width)&&this.j&&(this.j.style.width=Hd(Number(c)||c)),(a=a.height)&&this.j&&(this.j.style.height=Hd(Number(a)||a))};
function vs(a){var b;a.config&&!0!==a.config.loaded&&(a.config.loaded=!0,!a.config.args||"0"!==a.config.args.autoplay&&0!==a.config.args.autoplay&&!1!==a.config.args.autoplay?a.api.loadVideoByPlayerVars(null!==(b=a.config.args)&&void 0!==b?b:null):a.api.cueVideoByPlayerVars(a.config.args))}
function ys(a){var b=!0,c=zs(a);c&&a.config&&(a=As(a),b=gr(c,"version")===a);return b&&!!A("yt.player.Application.create")}
function us(a){if(!a.h()&&!a.Z){var b=ys(a);if(b&&"html5"===(zs(a)?"html5":null))a.ga="html5",a.isReady()||Bs(a);else if(Cs(a),a.ga="html5",b&&a.l&&a.m)a.m.appendChild(a.l),Bs(a);else{a.config&&(a.config.loaded=!0);var c=!1;a.A=function(){c=!0;var d=Ds(a,"player_bootstrap_method")?A("yt.player.Application.createAlternate")||A("yt.player.Application.create"):A("yt.player.Application.create");var e=a.config?ws(a.config):void 0;d&&d(a.m,e,a.webPlayerContextConfig);Bs(a)};
a.Z=!0;b?a.A():(kr(As(a),a.A),(b=Es(a))&&ks(b),Fs(a)&&!c&&y("yt.player.Application.create",null,void 0))}}}
function zs(a){var b=ud(a.elementId);!b&&a.j&&a.j.querySelector&&(b=a.j.querySelector("#"+a.elementId));return b}
function Bs(a){var b;if(!a.h()){var c=zs(a),d=!1;c&&c.getApiInterface&&c.getApiInterface()&&(d=!0);d?(a.Z=!1,!Ds(a,"html5_remove_not_servable_check_killswitch")&&(null===c||void 0===c?0:c.isNotServable)&&a.config&&(null===c||void 0===c?0:c.isNotServable(null===(b=a.config.args)||void 0===b?void 0:b.video_id))||Gs(a)):a.mb=Lh(function(){Bs(a)},50)}}
function Gs(a){rs(a);a.Ua=!0;var b=zs(a);if(b){a.u=Hs(a,b,"addEventListener");a.Ea=Hs(a,b,"removeEventListener");var c=b.getApiInterface();c=c.concat(b.getInternalApiInterface());for(var d=a.api,e=0;e<c.length;e++){var f=c[e];d[f]||(d[f]=Hs(a,b,f))}}for(var g in a.i)a.i.hasOwnProperty(g)&&a.u&&a.u(g,a.i[g]);vs(a);a.Fa&&a.Fa(a.api);a.S.ka("onReady",a.api)}
function Hs(a,b,c){var d=b[c];return function(){var e=Fa.apply(0,arguments);try{return a.lastError=null,d.apply(b,e)}catch(f){"sendAbandonmentPing"!==c&&(f.params=c,a.lastError=f,Ln(f))}}}
function rs(a){a.Ua=!1;if(a.Ea)for(var b in a.i)a.i.hasOwnProperty(b)&&a.Ea(b,a.i[b]);for(var c in a.L)a.L.hasOwnProperty(c)&&Mh(Number(c));a.L={};a.u=null;a.Ea=null;b=a.api;for(var d in b)b.hasOwnProperty(d)&&(b[d]=null);b.addEventListener=function(e,f){a.addEventListener(e,f)};
b.removeEventListener=function(e,f){a.removeEventListener(e,f)};
b.destroy=function(){a.dispose()};
b.getLastError=function(){return a.getLastError()};
b.getPlayerType=function(){return a.getPlayerType()};
b.getCurrentVideoConfig=function(){return a.Va};
b.loadNewVideoConfig=function(e){a.loadNewVideoConfig(e)};
b.isReady=function(){return a.isReady()}}
m.isReady=function(){return this.Ua};
function ss(a){a.addEventListener("WATCH_LATER_VIDEO_ADDED",function(b){Mi("WATCH_LATER_VIDEO_ADDED",b)});
a.addEventListener("WATCH_LATER_VIDEO_REMOVED",function(b){Mi("WATCH_LATER_VIDEO_REMOVED",b)});
a.addEventListener("onAdAnnounce",function(b){Mi("a11y-announce",b)})}
m.addEventListener=function(a,b){var c=this,d=xs(this,b);d&&(0<=cb(this.Lb,a)||this.i[a]||(b=Is(this,a),this.u&&this.u(a,b)),this.S.subscribe(a,d),"onReady"===a&&this.isReady()&&Lh(function(){d(c.api)},0))};
m.removeEventListener=function(a,b){this.h()||(b=xs(this,b))&&Vf(this.S,a,b)};
function xs(a,b){var c=b;if("string"===typeof b){if(a.Ga[b])return a.Ga[b];c=function(){var d=Fa.apply(0,arguments),e=A(b);if(e)try{e.apply(x,d)}catch(f){Kn(f)}};
a.Ga[b]=c}return c?c:null}
function Is(a,b){var c="ytPlayer"+b+a.M;a.i[b]=c;x[c]=function(d){var e=Lh(function(){if(!a.h()){a.S.ka(b,null!==d&&void 0!==d?d:void 0);var f=a.L,g=String(e);g in f&&delete f[g]}},0);
pb(a.L,String(e))};
return c}
m.getPlayerType=function(){return this.ga||(zs(this)?"html5":null)};
m.getLastError=function(){return this.lastError};
function Cs(a){a.cancel();rs(a);a.ga=null;a.config&&(a.config.loaded=!1);var b=zs(a);b&&(ys(a)||!Fs(a)?a.l=b:(b&&b.destroy&&b.destroy(),a.l=null));if(a.m)for(a=a.m;b=a.firstChild;)a.removeChild(b)}
m.cancel=function(){this.A&&qr(As(this),this.A);Mh(this.mb);this.Z=!1};
m.I=function(){Cs(this);if(this.l&&this.config&&this.l.destroy)try{this.l.destroy()}catch(b){Kn(b)}this.Ga=null;for(var a in this.i)this.i.hasOwnProperty(a)&&(x[this.i[a]]=null);this.Va=this.config=this.api=null;delete this.m;delete this.j;I.prototype.I.call(this)};
function Fs(a){var b,c;a=null===(c=null===(b=a.config)||void 0===b?void 0:b.args)||void 0===c?void 0:c.fflags;return!!a&&-1!==a.indexOf("player_destroy_old_version=true")}
function As(a){return a.webPlayerContextConfig?a.webPlayerContextConfig.jsUrl:(a=a.config.assets)?a.js:""}
function Es(a){return a.webPlayerContextConfig?a.webPlayerContextConfig.cssUrl:(a=a.config.assets)?a.css:""}
function Ds(a,b){var c;if(a.webPlayerContextConfig)var d=a.webPlayerContextConfig.serializedExperimentFlags;else if(null===(c=a.config)||void 0===c?0:c.args)d=a.config.args.fflags;return"true"===fi(d||"","&")[b]}
function ws(a){for(var b={},c=q(Object.keys(a)),d=c.next();!d.done;d=c.next()){d=d.value;var e=a[d];b[d]="object"===typeof e?sb(e):e}return b}
;var Js={},Ks="player_uid_"+(1E9*Math.random()>>>0);function Ls(a,b,c){var d="player";c=void 0===c?!0:c;d="string"===typeof d?ud(d):d;var e=Ks+"_"+Pa(d),f=Js[e];if(f&&c)return Ms(a,b)?f.api.loadVideoByPlayerVars(a.args||null):f.loadNewVideoConfig(a),f.api;f=new qs(d,e,a,b);Js[e]=f;Mi("player-added",f.api);Yd(f,function(){delete Js[f.getId()]});
return f.api}
function Ms(a,b){return b&&b.serializedExperimentFlags?b.serializedExperimentFlags.includes("web_player_remove_playerproxy=true"):a&&a.args&&a.args.fflags?a.args.fflags.includes("web_player_remove_playerproxy=true"):!1}
;var Ns=null,Os=null,Ps=null;function Qs(){var a=Ns.getVideoData(1);a=a.title?a.title+" - YouTube":"YouTube";document.title!==a&&(document.title=a)}
;function Rs(a,b,c){a="ST-"+Vb(a).toString(36);b=b?ac(b):"";c=c||5;qo()&&Bj(a,b,c)}
;function Ss(a,b,c){b=void 0===b?{}:b;c=void 0===c?!1:c;var d=B("EVENT_ID");d&&(b.ei||(b.ei=d));if(b){d=a;var e=void 0===e?!0:e;var f=B("VALID_SESSION_TEMPDATA_DOMAINS",[]),g=Yb(window.location.href);g&&f.push(g);g=Yb(d);if(0<=cb(f,g)||!g&&0==d.lastIndexOf("/",0))if(K("autoescape_tempdata_url")&&(f=document.createElement("a"),Qb(f,d),d=f.href),d&&(d=Zb(d),f=d.indexOf("#"),d=0>f?d:d.substr(0,f)))if(e&&!b.csn&&(b.itct||b.ved)&&(b=Object.assign({csn:bo()},b)),h){var h=parseInt(h,10);isFinite(h)&&0<h&&
Rs(d,b,h)}else Rs(d,b)}if(c)return!1;if((window.ytspf||{}).enabled)spf.navigate(a);else{var k=void 0===k?{}:k;var l=void 0===l?"":l;var n=void 0===n?window:n;c=n.location;a=bc(a,k)+l;var t=void 0===t?Gd:t;a:{t=void 0===t?Gd:t;for(k=0;k<t.length;++k)if(l=t[k],l instanceof Ed&&l.isValid(a)){t=new od(a,md);break a}t=void 0}c.href=qd(t||pd)}return!0}
;y("yt.setConfig",oh,void 0);y("yt.config.set",oh,void 0);y("yt.setMsg",ho,void 0);y("yt.msgs.set",ho,void 0);y("yt.logging.errors.log",Kn,void 0);
y("writeEmbed",function(){var a=B("PLAYER_CONFIG",void 0);if(!a){var b=B("PLAYER_VARS",void 0);b&&(a={args:b})}yo(!0);"gvn"===a.args.ps&&(document.body.style.backgroundColor="transparent");a.attrs||(a.attrs={width:"100%",height:"100%",id:"video-player"});var c=document.referrer;b=B("POST_MESSAGE_ORIGIN");window!==window.top&&c&&c!==document.URL&&(a.args.loaderUrl=c);K("embeds_js_api_set_1p_cookie")&&(c=ki(window.location.href),c.embedsTokenValue&&(a.args.embedsTokenValue=c.embedsTokenValue));dq();
if((c=B("WEB_PLAYER_CONTEXT_CONFIGS",void 0))&&"WEB_PLAYER_CONTEXT_CONFIG_ID_EMBEDDED_PLAYER"in c){c=c.WEB_PLAYER_CONTEXT_CONFIG_ID_EMBEDDED_PLAYER;if(!c.serializedForcedExperimentIds){var d=ki(window.location.href);d.forced_experiments&&(c.serializedForcedExperimentIds=d.forced_experiments)}Ns=Ls(a,c,!1)}else Ns=Ls(a);Ns.addEventListener("onVideoDataChange",Qs);a=B("POST_MESSAGE_ID","player");B("ENABLE_JS_API")?Ps=new as(Ns):B("ENABLE_POST_API")&&"string"===typeof a&&"string"===typeof b&&(Os=new hs(window.parent,
a,b),Ps=new es(Ns,Os.connection));tr();K("ytidb_create_logger_embed_killswitch")||mk();K("flush_gel_on_teardown")&&(a={},Lr||(Lr=new Kr),Lr.install((a.flush_logs={la:function(){ej()}},a)));
K("networkless_logging_web_embedded")&&(K("embeds_web_enable_new_nwl")?Vm():cn());K("embeds_enable_ua_ch_polyfill")&&Tr();K("ytidb_clear_embedded_player")&&Vh.N(function(){if(!Iq){var e={pb:{feedbackEndpoint:Po(Cq),modifyChannelNotificationPreferenceEndpoint:Po(Dq),playlistEditEndpoint:Po(Eq),subscribeEndpoint:Po(Aq),unsubscribeEndpoint:Po(Bq),webPlayerShareEntityServiceEndpoint:Po(Fq)}},f=K("web_enable_client_location_service")?Jo.getInstance():void 0,g={};f&&(g.client_location=f);if(void 0===h){zo.h||
(zo.h=new zo);var h=zo.h}if(void 0===k){Hq.h||(Hq.h=new Hq);var k=Hq.h}oq(e,k,h,g);Iq=nq.h}dr()})},void 0);
var Ts=Ah(function(){kq();var a=Fj.getInstance(),b=Ij(119),c=1<window.devicePixelRatio;if(document.body&&df(document.body,"exp-invert-logo"))if(c&&!df(document.body,"inverted-hdpi")){var d=document.body;if(d.classList)d.classList.add("inverted-hdpi");else if(!df(d,"inverted-hdpi")){var e=bf(d);cf(d,e+(0<e.length?" inverted-hdpi":"inverted-hdpi"))}}else!c&&df(document.body,"inverted-hdpi")&&ef();if(b!=c){b="f"+(Math.floor(119/31)+1);d=Jj(b)||0;d=c?d|67108864:d&-67108865;0==d?delete Ej[b]:(c=d.toString(16),
Ej[b]=c.toString());c=!0;K("web_secure_pref_cookie_killswitch")&&(c=!1);b=a.h;d=[];for(var f in Ej)d.push(f+"="+encodeURIComponent(String(Ej[f])));Bj(b,d.join("&"),63072E3,a.i,c)}Sq.h||(Sq.h=new Sq);a=Sq.h;f=16623;var g=void 0===g?{}:g;Object.values(io).includes(f)||(Ln(new sk("createClientScreen() called with a non-page VE",f)),f=83769);g.isHistoryNavigation||a.h.push({rootVe:f,key:g.key||""});a.o=[];a.D=[];g.sb?Vq(a,f,g):Wq(a,f,g)}),Us=Ah(function(){Ns&&Ns.sendAbandonmentPing&&Ns.sendAbandonmentPing();
B("PL_ATT")&&zr.dispose();for(var a=0,b=rr.length;a<b;a++)Vh.U(rr[a]);rr.length=0;pr("//static.doubleclick.net/instream/ad_status.js");sr=!1;oh("DCLKSTAT",0);Xd(Ps,Os);Ns&&(Ns.removeEventListener("onVideoDataChange",Qs),Ns.destroy())});
window.addEventListener?(window.addEventListener("load",Ts),window.addEventListener("unload",Us)):window.attachEvent&&(window.attachEvent("onload",Ts),window.attachEvent("onunload",Us));Wa("yt.abuse.player.botguardInitialized",A("yt.abuse.player.botguardInitialized")||Ar);Wa("yt.abuse.player.invokeBotguard",A("yt.abuse.player.invokeBotguard")||Br);Wa("yt.abuse.dclkstatus.checkDclkStatus",A("yt.abuse.dclkstatus.checkDclkStatus")||ur);
Wa("yt.player.exports.navigate",A("yt.player.exports.navigate")||Ss);Wa("yt.util.activity.init",A("yt.util.activity.init")||Xh);Wa("yt.util.activity.getTimeSinceActive",A("yt.util.activity.getTimeSinceActive")||$h);Wa("yt.util.activity.setTimestamp",A("yt.util.activity.setTimestamp")||Yh);}).call(this);
