#include "StateReader.h"

StateReader::StateReader()
{
 sprintf(_fileName, "%s.txt", STATE_BASE_NAME);
}

StateReader::StateReader(const char* fileName)
{
 sprintf(_fileName, "%s.txt", fileName);
}

void StateReader::setFileName(const char* fileName)
{
 sprintf(_fileName, "%s.txt", fileName);
}

bool StateReader::readObjectsFromFile(World* world)
{
	std::cout << "filename: " << _fileName << std::endl;
  ifstream file;
  file.open(_fileName);
  
  if (file.fail()) {
  	cout << "Failed to open file. Objects were not read from file." << endl;
  	return false;
  }

  bool success = readFromFile(file, world);
  file.close();
  return success;
}

bool StateReader::readFromFile(ifstream& file, World* world)
{
	world->clearObjs();

  int type;
  
  while (!file.eof()) {
    file >> type;
    switch (type)
    {
      case THREAD_CONSTRAINED:
        {
          ThreadConstrained* thread = new ThreadConstrained(file);
          world->addThread(thread);
          break;
        }
      case CAPSULE:
        {
          Capsule* obj = new Capsule(file);
          world->addEnvObj(obj);
          break;
        }
      case CURSOR:
        {
          Cursor* obj = new Cursor(file);
          world->addEnvObj(obj);
          break;
        }
      case END_EFFECTOR:
        {
          EndEffector* obj = new EndEffector(file);
          world->addEnvObj(obj);
          break;
        }
      case INFINITE_PLANE:
        {
          InfinitePlane* obj = new InfinitePlane(file);
          world->addEnvObj(obj);
          break;
        }
      case TEXTURED_SPHERE:
        {
          TexturedSphere* obj = new TexturedSphere(file);
          world->addEnvObj(obj);
          break;
        }
    }
    if (type == NO_OBJECT) { break; }
  }

	vector<ThreadConstrained*> threads = *(world->getThreads());
  for (int i = 0; i < threads.size(); i++) {
    threads[i]->setWorld(world);
  }

  world->initializeThreadsInEnvironment();

  vector<EnvObject*> objects = *(world->getEnvObjs());
  for (int i = 0; i < objects.size(); i++) {
  	objects[i]->linkPointersFromInd(world);
  }
  
  return true;
}
