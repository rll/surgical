#include "StateRecorder.h"

StateRecorder::StateRecorder()
{
	sprintf(_fileName, "%s.txt", STATE_BASE_NAME);
}

StateRecorder::StateRecorder(const char* fileName)
{
	sprintf(_fileName, "%s.txt", fileName);
}

void StateRecorder::setFileName(const char* fileName)
{
	sprintf(_fileName, "%s.txt", fileName);
}

void StateRecorder::writeObjectsToFile(World* world)
{
  std::cout << "Writing to: " << _fileName << std::endl;

  ofstream file;
  file.precision(20);
  file.open(_fileName);

  writeToFile(file, world);

  file.close();

  std::cout << "Writing Done\n";
}

void StateRecorder::writeToFile(ofstream& file, World* world)
{
  vector<EnvObject*> objects = *(world->getEnvObjs());
  vector<ThreadConstrained*> threads = *(world->getThreads());

  for (int i = 0; i < threads.size(); i++) {
    threads[i]->writeToFile(file);
  }
  
  for (int i = 0; i < objects.size(); i++) {
  	objects[i]->updateIndFromPointers(world);
    objects[i]->writeToFile(file);
  }
  
  file << NO_OBJECT << " ";
  file << "\n";
}
