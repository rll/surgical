#ifndef _Object_h
#define _Object_h

#include "ObjectTypes.h"

class Object
{
public:
  Object(object_type t)
  	: type(t)
  	{}
  
  virtual ~Object() {}

	object_type getType() { return type; }

protected:
  object_type type;
};

#endif //_Object_h
