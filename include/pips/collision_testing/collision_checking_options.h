#ifndef COLLISION_CHECKING_OPTIONS_H
#define COLLISION_CHECKING_OPTIONS_H

struct CCOptions
{
  bool get_details_ = true;
  bool full_details = true;
  
  CCOptions() {}
  CCOptions(bool get_details, bool full_details=true) : get_details_(get_details), full_details(full_details) {}
  
  operator bool() const
  {
      return get_details_; // Or false!
  }
};


#endif /* COLLISION_CHECKING_OPTIONS_H */
