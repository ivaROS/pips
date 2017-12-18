#ifndef COLLISION_CHECKING_OPTIONS_H
#define COLLISION_CHECKING_OPTIONS_H

struct CCOptions
{
  bool get_details_ = true;
  
  
  CCOptions() {}
  CCOptions(bool get_details) : get_details_(get_details) {}
  
  operator bool() const
  {
      return get_details_; // Or false!
  }
};


#endif /* COLLISION_CHECKING_OPTIONS_H */