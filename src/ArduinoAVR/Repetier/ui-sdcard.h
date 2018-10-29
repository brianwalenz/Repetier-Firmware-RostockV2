/*
  This file is part of Repetier-Firmware.

  Repetier-Firmware is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Repetier-Firmware is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <math.h>
#include <stdlib.h>
#include <inttypes.h>
#include <ctype.h>

#include "Repetier.h"


extern UIDisplay uid;



void
UIDisplay::scanSDcard(uint16_t filePos, char *filename) {
  dir_t   *p    = NULL;
  FatFile *root = sd.getvwd();
  FatFile  file;

  root->rewind();

  nFilesOnCard = 0;

#define SHOW_SCANSDCARD

#ifdef SHOW_SCANSDCARD
  Com::print("\n");
  Com::print("scanSDcard\n");
#endif

  while (file.openNext(root, O_READ)) {
    HAL::pingWatchdog();

    bool isDir = file.isDir();

    file.getName(tempLongFilename, LONG_FILENAME_LENGTH);
    file.close();

#ifdef SHOW_SCANSDCARD
    Com::print("scanSDcard -- '");
    Com::print(tempLongFilename);
    Com::print("'\n");
#endif

    //  Skip dot files.

    if ((tempLongFilename[0] == '.') && (tempLongFilename[1] != '.'))
      continue;

    //  If this is the file we want to get the name for, copy the name.

    if ((filename != NULL) && (nFilesOnCard == filePos)) {
      uint16_t  pos = 0;

      for (pos=0; tempLongFilename[pos] != 0; pos++)
        filename[pos] = tempLongFilename[pos];

      if (isDir)
        filename[pos++] = '/';

      filename[pos] = 0;
    }

    nFilesOnCard++;
  }

#ifdef SHOW_SCANSDCARD
  Com::print("scanSDcard ");
  Com::print(nFilesOnCard);
  Com::print("\n");
#endif
}





bool UIDisplay::isDirname(char *name) {

  while(*name)
    name++;

  name--;

  return(*name == '/');
}



void UIDisplay::goDir(char *name) {
  char *p = cwd;

  //  Skip to the end of the string.

  while (*p)
    p++;

  //  If no name supplied, move up one level.

  if (name == NULL) {
    if ((cwd[0] == '/') && (cwd[1] == 0))
      return;

    p--;  //  Move back off the NUL, now on a /
    p--;  //  Move back off the /.   now on the last letter in the directory name

    while (*p != '/')  //  Search back for the next /
      p--;

    *++p = 0;    //  and make it the end of the string.
  }

  //  Otherwise, go into the directory.

  else {
    while (*name)
      *p++ = *name++;

    *p = 0;
  }

  //  Now set the directory.

  sd.chdir(cwd);

  scanSDcard();
}




/** write file names at current position to lcd */
uint8_t
UIDisplay::sdrefresh(char cache[UI_ROWS][MAX_COLS + 1]) {
  FatFile   *root;
  FatFile    file;

  //if (nFilesOnCard == 0)
  //  scanSDcard();  //  necessary?  should be done when inserted.

  //  The menu shows:
  //    SELECT FILE TO PRINT
  //    [..]      --
  //    file1     -- enti == 0
  //    file2     -- enti == 1
  //
  uint8_t    rowi = 0;                    //  Which display row are we creating?
  uint8_t    enti = 0;                    //  Which file entry are we processing?

  // _menuTop   Which entry is the first to be displayed?
  // _menuPos   Which entry is highlighted?

#define SHOW_SDREFRESH

#ifdef SHOW_SDREFRESH
  Com::print("sdrefresh -- menuTop=");
  Com::print(_menuTop);
  Com::print(" menuPos=");
  Com::print(_menuPos);
  Com::print("\n");
#endif

  sd.chdir(cwd);

  root = sd.getvwd();
  root->rewind();

  //  If showing the first element, it's the usual header.

  if (_menuTop == 0) {
    uint8_t  col = 0;

    if      (_menuSel == 0)                 //  The first xolumn is either
      cache[rowi][col++] = CHAR_SELECTED;   //  SELECTED
    else if (_menuPos == 0)                 //  or
      cache[rowi][col++] = CHAR_SELECTOR;   //  has the selector on it
    else                                    //  or
      cache[rowi][col++]  = ' ';            //  doesn't.

    cache[rowi][col++]  = 0x7f;             //  The second column is always a back arrow.

    for (; col < 19; col++)
      cache[rowi][col] = pgm_read_byte(&page01_01[col]);

    cache[rowi][col++] = 0x7e;              //  The last column is always a forward arrow.

    rowi++;
  }

  //  If showing the first or second element, make the second element by
  //  'up dir' regardlss of if there is an up dir (greatly simplifies the
  //  logic when processing the button clicks).

  if ((_menuTop == 0) ||
      (_menuTop == 1)) {
    uint8_t col = 0;

    cache[rowi][col++] = (_menuPos == 1) ? CHAR_SELECTOR : ' ';
    cache[rowi][col++] = '[';
    cache[rowi][col++] = '.';
    cache[rowi][col++] = '.';
    cache[rowi][col++] = ']';
    
    while (col < MAX_COLS)
      cache[rowi][col++] = ' ';

    rowi++;
  }

  //  Iterate through files until we fill the list.

  while ((enti < nFilesOnCard) &&
         (rowi < UI_ROWS) &&
         (file.openNext(root, O_READ))) {

    HAL::pingWatchdog();

    bool isDir = file.isDir();

    file.getName(tempLongFilename, LONG_FILENAME_LENGTH);
    file.close();

#ifdef SHOW_SDREFRESH
    Com::print("sdrefresh -- enti=");
    Com::print(enti);
    Com::print(" '");
    Com::print(tempLongFilename);
    Com::print("'\n");
#endif

    //  Skip junk.

    if (tempLongFilename[0] == '.')
      continue; 

    //  Skip files that aren't displayed.  Can't skip them until we skip the garbage files above.
    //
    //  A menuTop of 0 means we're showing the page title as the first element, and we should skip no file names.
    //  A menuTop of 1 means we're showing [..] as the first element, and we should skip no file names.
    //  A menuTop of 2 means we're showing the first file as the first, and we should skip no file names.
    //  A menuTop of 3 ... and we should skip the first file (at enti==0).
    //
    if (enti + 2 < _menuTop) {
      enti++;
      continue;
    }

    //  Mark this file as selected?  The first file is 'enti 0', but is in the second menuPos (menuPos = 1)

    uint8_t col = 0;

    if (enti + 2 == _menuPos)
      cache[rowi][col++] = CHAR_SELECTOR;
    else
      cache[rowi][col++] = ' ';

    //  Add the filename.

    if (isDir)
      cache[rowi][col++] = '[';

    uint8_t   length = 0;

    while ((length < MAX_COLS - col - isDir) && (tempLongFilename[length] != 0))
      length++;

#ifdef SHOW_SDREFRESH
    Com::print("sdrefresh -- length '");
    Com::print(length);
    Com::print("'\n");
#endif

    for (uint8_t pos=0; ((col < MAX_COLS) && (pos < length)); pos++, col++)
      cache[rowi][col] = tempLongFilename[pos];

    if (isDir)
      cache[rowi][col++] = ']';

    while (col < MAX_COLS)
      cache[rowi][col++] = ' ';

    rowi++;
    enti++;
  }

#ifdef SHOW_SDREFRESH
  Com::print("sdrefresh return\n");
#endif

  return(rowi);
}

