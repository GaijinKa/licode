#ifndef BUILDING_NODE_EXTENSION
#define BUILDING_NODE_EXTENSION
#endif
#include <node.h>
#include "ExternalOutput.h"


using namespace v8;

ExternalOutput::ExternalOutput() {};
ExternalOutput::~ExternalOutput() {};

void ExternalOutput::Init(Handle<Object> target) {
  // Prepare constructor template
  Local<FunctionTemplate> tpl = FunctionTemplate::New(New);
  tpl->SetClassName(String::NewSymbol("ExternalOutput"));
  tpl->InstanceTemplate()->SetInternalFieldCount(1);
  // Prototype
  tpl->PrototypeTemplate()->Set(String::NewSymbol("close"), FunctionTemplate::New(close)->GetFunction());
  tpl->PrototypeTemplate()->Set(String::NewSymbol("init"), FunctionTemplate::New(init)->GetFunction());

  Persistent<Function> constructor = Persistent<Function>::New(tpl->GetFunction());
  target->Set(String::NewSymbol("ExternalOutput"), constructor);
}

Handle<Value> ExternalOutput::New(const Arguments& args) {
  HandleScope scope;

  ExternalOutput* obj = new ExternalOutput();
  obj->me = new erizo::ExternalOutput();

  obj->Wrap(args.This());

  return args.This();
}

Handle<Value> ExternalOutput::close(const Arguments& args) {
  HandleScope scope;

  ExternalOutput* obj = ObjectWrap::Unwrap<ExternalOutput>(args.This());
  erizo::ExternalOutput *me = (erizo::ExternalOutput*)obj->me;

  delete me;

  return scope.Close(Null());
}

Handle<Value> ExternalOutput::init(const Arguments& args) {
  HandleScope scope;

  ExternalOutput* obj = ObjectWrap::Unwrap<ExternalOutput>(args.This());
  erizo::ExternalOutput *me = (erizo::ExternalOutput*) obj->me;
  v8::String::Utf8Value param(args[0]->ToString());
  v8::String::Utf8Value param2(args[1]->ToString());
  v8::String::Utf8Value param3(args[2]->ToString());
  v8::String::Utf8Value param4(args[3]->ToString());
  v8::String::Utf8Value param5(args[4]->ToString());

// convert it to string
  std::string path = std::string(*param);
  std::string name = std::string(*param2);
  std::string room = std::string(*param3);
  std::string width = std::string(*param4);
  std::string height = std::string(*param5);

  int r = me->init(path, name, room, width, height);

  return scope.Close(Integer::New(r));
}

