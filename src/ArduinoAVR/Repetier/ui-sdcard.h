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
UIDisplay::updateSDFileCount() {
  dir_t   *p    = NULL;
  FatFile *root = sd.fat.vwd();
  FatFile  file;

  root->rewind();

  nFilesOnCard = 0;

  Com::print("\n");
  Com::print("updateSDFileCount\n");

  while (file.openNext(root, O_READ)) {
    HAL::pingWatchdog();

    bool isDir = file.isDir();

    file.getName(tempLongFilename, LONG_FILENAME_LENGTH);
    file.close();

    Com::print("updateSDFileCount -- '");
    Com::print(tempLongFilename);
    Com::print("'\n");

    if ((tempLongFilename[0] == '.') && (tempLongFilename[1] != '.'))
      continue;

    nFilesOnCard++;

    if (nFilesOnCard > 5000) // Arbitrary maximum, limited only by how long someone would scroll
      return;
  }

  Com::print("updateSDFileCount ");
  Com::print(nFilesOnCard);
  Com::print("\n");
}



//  MAKE PART OF UID
void getSDFilenameAt(uint16_t filePos, char *filename) {
  dir_t* p = NULL;
  FatFile *root = sd.fat.vwd();
  FatFile file;

  root->rewind();

  while (file.openNext(root, O_READ)) {
    HAL::pingWatchdog();

    bool isDir = file.isDir();

    file.getName(tempLongFilename, LONG_FILENAME_LENGTH);
    file.close();

    if ((tempLongFilename[0] == '.') && (tempLongFilename[1] != '.'))
      continue;

    if (filePos--)
      continue;

    uint16_t  pos = 0;

    for (pos=0; tempLongFilename[pos] != 0; pos++)
      filename[pos] = tempLongFilename[pos];

    if (isDir)
      filename[pos++] = '/';

    filename[pos] = 0;

    break;
  }
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

  sd.fat.chdir(cwd);

  updateSDFileCount();
}




/** write file names at current position to lcd */
uint8_t
UIDisplay::sdrefresh(char cache[UI_ROWS][MAX_COLS + 1]) {
  FatFile   *root;
  FatFile    file;

  //  The menu shows:
  //    [..]      --
  //    file1     -- enti == 0
  //    file2     -- enti == 1
  //
  uint8_t    rowi = 0;                    //  Which display row are we creating?
  uint8_t    enti = 0;                    //  Which file entry are we processing?

  // menuTop[menuLevel]   Which entry is the first to be displayed?
  // menuPos[menuLevel]   Which entry is highlighted?

  Com::print("sdrefresh -- menuTop=");
  Com::print(menuTop[menuLevel]);
  Com::print(" menuPos=");
  Com::print(menuPos[menuLevel]);
  Com::print("\n");

  sd.fat.chdir(cwd);

  root = sd.fat.vwd();
  root->rewind();

  //  If no active files. reset menuTop to be 'back'.

  if (nFilesOnCard == 0) {
    menuTop[menuLevel] = 0;
    menuPos[menuLevel] = 0;
  }

  //  If at the first or second entry, make the first displayed item be '..'

  if (menuTop[menuLevel] == 0) {
    col = 0;

    cache[rowi][col++] = (menuPos[menuLevel] == 0) ? CHAR_SELECTOR : ' ';
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

    Com::print("sdrefresh -- enti=");
    Com::print(enti);
    Com::print(" '");
    Com::print(tempLongFilename);
    Com::print("'\n");

    //  Skip junk.

    if (tempLongFilename[0] == '.' && tempLongFilename[1] != '.')
      continue; 

    //  Skip files that aren't displayed.  Can't skip them until we skip the garbage files above.
    //
    //  A menuTop of 0 means we're showing [..] as the first element, and we should skip no file names.
    //  A menuTop of 1 means we're showing the first file as the first, and we should skip no file names.
    //  A menuTop of 2 ... and we should skip the first file (at enti==0).
    //
    if (enti + 1 < menuTop[menuLevel]) {
      enti++;
      continue;
    }

    //  Mark this file as selected?  The first file is 'enti 0', but is in the second menuPos (menuPos = 1)

    col = 0;

    if (enti + 1 == menuPos[menuLevel])
      printCols[col++] = CHAR_SELECTOR;
    else
      printCols[col++] = ' ';

    //  Add the filename.

    if (isDir)
      printCols[col++] = '[';

    uint8_t   length = 0;

    while ((length < MAX_COLS - col - isDir) && (tempLongFilename[length] != 0))
      length++;

    Com::print("sdrefresh -- length '");
    Com::print(length);
    Com::print("'\n");

    for (uint8_t pos=0; ((col < MAX_COLS) && (pos < length)); pos++, col++)
      printCols[col] = tempLongFilename[pos];

    if (isDir)
      printCols[col++] = ']';

    while (col < MAX_COLS)
      printCols[col++] = ' ';

    for (uint8_t pos=0; pos<MAX_COLS; pos++)
      cache[rowi][pos] = printCols[pos];

    rowi++;
    enti++;
  }

  Com::print("sdrefresh return\n");

  return(rowi);
}

