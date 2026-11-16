#include <Arduino.h>
#include <esp_display_panel.hpp>

#include <lvgl.h>
#include "lvgl_v8_port.h"

#include <FS.h>
#include <SD.h>
#include <SPI.h>

#include <stdint.h>
#include <cstring>
#include <cstdlib>
#include <cmath>

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// V1.0:
// - Explicit SID/STAR transition selection
// - Procedures are shown only when present in the installed navdata
// - Live Feather sensor computer over ESP-NOW
// - Live GPS position/UTC on POS INIT
// - GPS position automatically drives LAT/LON FROM when available
// - DIRECT TO supports navdata waypoints or Boeing-format coordinates
// - PROGRESS shows DTG/DTK from GPS or POS INIT, with live GS/TRK/ETE/ETA when GPS is valid

using namespace esp_panel::drivers;
using namespace esp_panel::board;

// =====================================================
// COLORS
// =====================================================

#define CDU_BLACK        lv_color_hex(0x000000)
#define CDU_WHITE        lv_color_hex(0xFFFFFF)
#define CDU_MAGENTA      lv_color_hex(0xFF00D4)
#define CDU_DARK_GRAY    lv_color_hex(0x202020)
#define CDU_MEDIUM_GRAY  lv_color_hex(0x404040)
#define CDU_REVERSE_VIDEO lv_color_hex(0xB8B8B8)

// =====================================================
// PAGE / INPUT SYSTEM
// =====================================================

enum class PageId : intptr_t {
    CDU_MENU,
    IDENT,
    POS_INIT,
    INDEX,
    DEP_ARR,
    DEPARTURES,
    ARRIVALS,
    SID_TRANSITIONS,
    STAR_TRANSITIONS,
    ROUTE,
    LEGS,
    DIRECT_TO,
    PROGRESS,
    LAT_LON,
    NAV_DATA,
    STATUS,
    SELECT_WPT
};

enum class InputTarget : intptr_t {
    NONE,
    SCRATCHPAD,
    ORIGIN,
    DESTINATION,
    WAYPOINT,
    NAV_LOOKUP,
    LEGS_INSERT,
    DIRECT_TO,
    LATLON_FROM,
    LATLON_TO
};

enum class KeyboardAction : intptr_t {
    CLEAR,
    DELETE_CHAR,
    CANCEL,
    ENTER
};

enum class ProcedureAction : intptr_t {
    SELECT_SID_TEST1,
    SELECT_SID_RUNWAY_27L,
    SELECT_SID_TRANSITION_PTW,
    CLEAR_DEPARTURE,
    SELECT_STAR_TEST2,
    SELECT_STAR_TRANSITION_MXE,
    SELECT_APPROACH_ILS27R,
    SELECT_APPROACH_RNAV27R,
    CLEAR_ARRIVAL
};

enum class RouteLegSource : uint8_t {
    SID_ARD,
    SID_PTW,
    MANUAL,
    INSERTED,
    STAR_MXE,
    STAR_PADRZ,
    APPROACH,
    SD_DEPARTURE,
    SD_ARRIVAL
};

enum class NavType : uint8_t {
    AIRPORT,
    FIX,
    VOR,
    NDB
};

struct NavRecord {
    const char *ident;
    NavType type;
};

// =====================================================
// BUILT-IN TEST NAV DATABASE
//
// This is only the development database.
// The SD-card version will replace this array later.
// =====================================================

static const NavRecord NAV_DATABASE[] = {
    // Airports
    {"KPNE", NavType::AIRPORT},
    {"KPHL", NavType::AIRPORT},
    {"KJFK", NavType::AIRPORT},
    {"KLGA", NavType::AIRPORT},
    {"KEWR", NavType::AIRPORT},
    {"KABE", NavType::AIRPORT},
    {"KMDT", NavType::AIRPORT},
    {"KBWI", NavType::AIRPORT},
    {"KDCA", NavType::AIRPORT},
    {"KIAD", NavType::AIRPORT},
    {"KILG", NavType::AIRPORT},
    {"KACY", NavType::AIRPORT},

    // VORs
    {"PTW", NavType::VOR},
    {"MXE", NavType::VOR},
    {"ARD", NavType::VOR},
    {"PHL", NavType::VOR},
    {"EMI", NavType::VOR},
    {"DQN", NavType::VOR},
    {"OTT", NavType::VOR},
    {"JFK", NavType::VOR},

    // Fixes
    {"CAMRN", NavType::FIX},
    {"HODAA", NavType::FIX},
    {"MADDY", NavType::FIX},
    {"PADRZ", NavType::FIX},
    {"BIGGY", NavType::FIX},
    {"ZIMMZ", NavType::FIX},
    {"LENDY", NavType::FIX},
    {"BRIXX", NavType::FIX}
};

constexpr size_t NAV_DATABASE_COUNT =
    sizeof(NAV_DATABASE) / sizeof(NAV_DATABASE[0]);

// =====================================================
// SD / FAA CIFP + WORLD NAVDATA
// =====================================================

// Waveshare ESP32-S3-Touch-LCD-4.3 TF-card SPI pins.
constexpr int SD_SPI_MOSI = 11;
constexpr int SD_SPI_SCK = 12;
constexpr int SD_SPI_MISO = 13;
// GPIO10 is the ESP32-S3 default SS pin, but on this Waveshare board
// GPIO10 is LCD blue data bit B7. Using SD.begin() with its default SS pin
// forces that LCD line high and makes the whole CDU appear blue.
//
// The real SD chip-select is controlled by CH422G EXIO4, so use an otherwise
// unused GPIO only as a harmless dummy CS for the Arduino SD library.
constexpr int SD_SPI_DUMMY_CS = 6;
constexpr int SD_CS_EXIO = 4;

constexpr size_t SD_INDEX_RECORD_SIZE = 27;

const char AIRPORT_INDEX_PATH[] = "/NAVDATA/airports.idx";
const char AIRPORT_DATA_PATH[] = "/NAVDATA/airports.dat";
const char POINT_INDEX_PATH[] = "/NAVDATA/points.idx";
const char POINT_DATA_PATH[] = "/NAVDATA/points.dat";
const char PROCEDURE_INDEX_PATH[] = "/NAVDATA/procedures.idx";
const char PROCEDURE_DATA_PATH[] = "/NAVDATA/procedures.dat";
const char RUNWAY_INDEX_PATH[] = "/NAVDATA/runways.idx";
const char RUNWAY_DATA_PATH[] = "/NAVDATA/runways.dat";
const char CYCLE_PATH[] = "/NAVDATA/cycle.txt";

bool sdMounted = false;
bool navDataReady = false;

char navCycle[8] = "----";
char navStatus[24] = "NO CARD";
char navEffective[24] = "----";
char navExpires[24] = "----";

uint32_t navAirportIndexCount = 0;
uint32_t navPointIndexCount = 0;
uint32_t navProcedureAirportCount = 0;

struct SdIndexHit {
    uint32_t offset;
    uint32_t count;
};

struct NavCandidate {
    char ident[9];
    NavType type;
    char region[9];
    char icao[5];
    int32_t latitudeE7;
    int32_t longitudeE7;
    char name[40];
};

// =====================================================
// FMC STATE
// =====================================================

constexpr int MAX_ROUTE_LEGS = 32;
constexpr int MAX_MATCHES = 8;

char routeOrigin[8] = "";
char routeDestination[8] = "";

int32_t routeOriginLatitudeE7 = 0;
int32_t routeOriginLongitudeE7 = 0;
int32_t routeDestinationLatitudeE7 = 0;
int32_t routeDestinationLongitudeE7 = 0;

// Manually-entered enroute waypoints are stored separately from
// procedure-generated legs. routeLegs[] is rebuilt whenever a SID,
// STAR, approach, or manual waypoint changes.
char manualRouteLegs[MAX_ROUTE_LEGS][8] = {};
int manualRouteLegCount = 0;

char routeLegs[MAX_ROUTE_LEGS][8] = {};
RouteLegSource routeLegSources[MAX_ROUTE_LEGS] = {};
int routeLegManualIndices[MAX_ROUTE_LEGS] = {};
int routeLegInsertionIndices[MAX_ROUTE_LEGS] = {};
int routeLegCount = 0;
int activeLegIndex = 0;

// LEGS-page insertions survive redraws and route rebuilds. Each inserted
// waypoint is anchored immediately before the selected route leg, or
// immediately before the destination.
constexpr int MAX_ROUTE_INSERTIONS = 12;

struct RouteInsertion {
    bool active;
    char ident[8];
    RouteLegSource anchorSource;
    int anchorManualIndex;
    bool beforeDestination;
};

RouteInsertion routeInsertions[MAX_ROUTE_INSERTIONS] = {};
int pendingLegInsertRouteIndex = -2;
bool pendingInsertionClosesDiscontinuity = false;

// A deletion creates a route discontinuity at this display position.
// -1 means there is no discontinuity.
int routeDiscontinuityIndex = -1;

// Development-procedure leg suppression. This allows one individual
// test-procedure leg to be removed without deselecting the full procedure.
bool omitSidArd = false;
bool omitSidPtw = false;
bool omitStarMxe = false;
bool omitStarPadrz = false;
bool omitApproachLeg = false;

bool routeModified = false;
bool routeExecuted = false;

// DIRECT TO target. A target becomes active as soon as a valid database
// waypoint or coordinate entry is accepted.
struct DirectToState {
    bool valid = false;
    char ident[16] = "";
    char entry[32] = "";
    char display[40] = "";
    double latitude = 0.0;
    double longitude = 0.0;
};

DirectToState directToTarget = {};

// Development-only KPHL procedure selections. These are deliberately
// marked TEST data and will be replaced by SD-card navdata later.
bool sidTest1Selected = false;
bool sidRunway27LSelected = false;
bool sidTransitionPTWSelected = false;

bool starTest2Selected = false;
bool starTransitionMXESelected = false;

enum class ApproachSelection : uint8_t {
    NONE,
    ILS27R,
    RNAV27R
};

ApproachSelection selectedApproach = ApproachSelection::NONE;


// =====================================================
// SD PROCEDURE SELECTION STATE
// =====================================================

constexpr int MAX_RUNWAY_OPTIONS = 20;
constexpr int MAX_PROCEDURE_OPTIONS = 30;
constexpr int MAX_TRANSITION_OPTIONS = 20;
constexpr int MAX_PROCEDURE_LEGS = 24;
constexpr int PROC_ROWS_PER_PAGE = 4;
constexpr int TRANS_ROWS_PER_PAGE = 5;

char departureRunways[MAX_RUNWAY_OPTIONS][8] = {};
char departureSids[MAX_PROCEDURE_OPTIONS][9] = {};
char departureTransitions[MAX_TRANSITION_OPTIONS][9] = {};

char arrivalRunways[MAX_RUNWAY_OPTIONS][8] = {};
char arrivalStars[MAX_PROCEDURE_OPTIONS][9] = {};
char arrivalTransitions[MAX_TRANSITION_OPTIONS][9] = {};

int departureRunwayCount = 0;
int departureSidCount = 0;
int departureTransitionCount = 0;

int arrivalRunwayCount = 0;
int arrivalStarCount = 0;
int arrivalTransitionCount = 0;

int departurePageIndex = 0;
int arrivalPageIndex = 0;
int sidTransitionPageIndex = 0;
int starTransitionPageIndex = 0;

char loadedDepartureAirport[8] = "";
char loadedArrivalAirport[8] = "";

char selectedDepartureRunway[8] = "";
char selectedSid[9] = "";
char selectedSidTransition[9] = "";

char selectedArrivalRunway[8] = "";
char selectedStar[9] = "";
char selectedStarTransition[9] = "";

char selectedDepartureLegs[MAX_PROCEDURE_LEGS][8] = {};
bool selectedDepartureLegEnabled[MAX_PROCEDURE_LEGS] = {};
int selectedDepartureLegCount = 0;

char selectedArrivalLegs[MAX_PROCEDURE_LEGS][8] = {};
bool selectedArrivalLegEnabled[MAX_PROCEDURE_LEGS] = {};
int selectedArrivalLegCount = 0;

// =====================================================
// MANUAL LAT/LON NAVIGATION STATE
// =====================================================

struct GeoPosition {
    bool valid;
    double latitude;
    double longitude;
    char entry[24];
    char display[32];
};

GeoPosition manualFromPosition = {};
GeoPosition manualToPosition = {};

// =====================================================
// LIVE SENSOR COMPUTER / ESP-NOW STATE
// =====================================================

constexpr uint8_t SENSOR_ESPNOW_CHANNEL = 6;
constexpr uint32_t SENSOR_PACKET_TIMEOUT_MS = 1500;
constexpr uint32_t SENSOR_PAGE_REFRESH_MS = 1000;

bool sensorEspNowReady = false;
portMUX_TYPE sensorEspNowMux = portMUX_INITIALIZER_UNLOCKED;

struct LiveSensorData {
    uint32_t sequence = 0;
    uint32_t featherUptimeMs = 0;

    bool gpsOnline = false;
    bool gpsFix = false;
    uint8_t satellites = 0;
    double latitude = 0.0;
    double longitude = 0.0;
    float groundSpeedKt = 0.0F;
    float trackDeg = 0.0F;
    float gpsAltitudeM = 0.0F;

    bool utcValid = false;
    uint8_t utcHour = 0;
    uint8_t utcMinute = 0;
    uint8_t utcSecond = 0;

    bool bmeOnline = false;
    float pressureHpa = 0.0F;
    float temperatureC = 0.0F;
    float humidityPercent = 0.0F;
    float baroAltitudeM = 0.0F;

    bool bnoOnline = false;
    float rollDeg = 0.0F;
    float pitchDeg = 0.0F;
    float headingDeg = 0.0F;
};

LiveSensorData liveSensors = {};
uint32_t lastSensorPacketMs = 0;
uint32_t lastSensorPageRefreshMs = 0;
uint32_t validSensorPacketCount = 0;
uint32_t badSensorPacketCount = 0;

char sensorRxBuffer[384] = {};
size_t sensorRxLength = 0;
volatile bool sensorEspNowFrameReady = false;

int legsPageIndex = 0;

char scratchpadText[32] = "";

// FMC scratchpad messages are protected: alphabetic/numeric keys do
// nothing until DEL is pressed. Pilot-entered text is not protected.
bool scratchpadMessageActive = false;

char lastLookupIdent[9] = "";
char lastLookupType[16] = "";
char lastLookupName[40] = "";
int32_t lastLookupLatitudeE7 = 0;
int32_t lastLookupLongitudeE7 = 0;
bool lastLookupValid = false;

// Duplicate-ident / selection support. Candidates can come from either
// the indexed SD database or the small built-in fallback database.
NavCandidate navMatches[MAX_MATCHES] = {};
int navMatchCount = 0;
InputTarget pendingMatchTarget = InputTarget::NONE;

// =====================================================
// PAGE STATE
// =====================================================

PageId currentPage = PageId::CDU_MENU;
PageId requestedPage = PageId::CDU_MENU;

bool redrawRequested = false;
uint32_t keyboardClosedAtMs = 0;

// =====================================================
// KEYBOARD STATE
// =====================================================

lv_obj_t *keyboardOverlay = nullptr;
lv_obj_t *keyboardTextArea = nullptr;
lv_obj_t *keyboardObject = nullptr;

InputTarget activeInputTarget = InputTarget::NONE;
// =====================================================
// POSITION INITIALIZATION DATA
// KPHL airport reference position, rounded to 0.1 minute
// =====================================================

const char LAST_POSITION_AIRPORT[] = "KPHL";
const char LAST_POSITION_LAT[] = "N39 52.7";
const char LAST_POSITION_LON[] = "W075 13.8";

char irsPosition[32] = "";
bool irsPositionSet = false;

// =====================================================
// FORWARD DECLARATIONS
// =====================================================

void renderPage(PageId page);
void requestPage(PageId page);

void serviceSensorBus();
void serviceSensorPageRefresh();
bool sensorComputerOnline();
bool liveGpsPositionAvailable();

void openKeyboard(InputTarget target);
void closeKeyboard();
void commitKeyboardInput();

void setScratchpad(const char *message);
void setScratchpadEntry(const char *entry);
void clearScratchpad();
bool scratchpadDeleteArmed();

bool initializeSdNavData(Board *board);
bool findSdIndexHit(const char *path, const char *key, SdIndexHit &hit);
int findNavMatches(const char *ident, InputTarget target);
void applyNavCandidate(int candidateIndex, InputTarget target);

void rebuildRouteLegs();
void insertRouteLegBefore(int routeIndex, const char *ident);
void deleteRouteLegAt(int routeIndex);
void legWaypointEvent(lv_event_t *event);
void discontinuityEvent(lv_event_t *event);
void validateNavEntry(const char *enteredText, InputTarget target);
void clearDepartureSelection();
void clearArrivalSelection();

// =====================================================
// TEXT HELPERS
// =====================================================

const char *navTypeText(NavType type)
{
    switch (type) {
        case NavType::AIRPORT:
            return "AIRPORT";

        case NavType::FIX:
            return "FIX";

        case NavType::VOR:
            return "VOR";

        case NavType::NDB:
            return "NDB";
    }

    return "UNKNOWN";
}

void copyUppercase(
    char *destination,
    size_t destinationSize,
    const char *source
) {
    String value = source;

    value.trim();
    value.toUpperCase();

    value.toCharArray(
        destination,
        destinationSize
    );
}

void setScratchpad(const char *message)
{
    strncpy(
        scratchpadText,
        message,
        sizeof(scratchpadText) - 1
    );

    scratchpadText[
        sizeof(scratchpadText) - 1
    ] = '\0';

    scratchpadMessageActive = true;
}

void setScratchpadEntry(const char *entry)
{
    strncpy(
        scratchpadText,
        entry,
        sizeof(scratchpadText) - 1
    );

    scratchpadText[
        sizeof(scratchpadText) - 1
    ] = '\0';

    scratchpadMessageActive = false;
}

void clearScratchpad()
{
    scratchpadText[0] = '\0';
    scratchpadMessageActive = false;
}

bool scratchpadDeleteArmed()
{
    return (
        !scratchpadMessageActive &&
        strcmp(scratchpadText, "DELETE") == 0
    );
}


// =====================================================
// ROUTE / PROCEDURE HELPERS
// =====================================================

void appendComputedRouteLeg(
    const char *ident,
    RouteLegSource source,
    int manualIndex = -1,
    int insertionIndex = -1
)
{
    if (
        ident == nullptr ||
        strlen(ident) == 0 ||
        routeLegCount >= MAX_ROUTE_LEGS
    ) {
        return;
    }

    strncpy(
        routeLegs[routeLegCount],
        ident,
        sizeof(routeLegs[routeLegCount]) - 1
    );

    routeLegs[routeLegCount][
        sizeof(routeLegs[routeLegCount]) - 1
    ] = '\0';

    routeLegSources[routeLegCount] = source;
    routeLegManualIndices[routeLegCount] = manualIndex;
    routeLegInsertionIndices[routeLegCount] = insertionIndex;

    routeLegCount++;
}


void insertComputedRouteLegAt(
    int position,
    const char *ident,
    int insertionIndex
)
{
    if (
        ident == nullptr ||
        strlen(ident) == 0 ||
        routeLegCount >= MAX_ROUTE_LEGS
    ) {
        return;
    }

    if (position < 0) {
        position = 0;
    }

    if (position > routeLegCount) {
        position = routeLegCount;
    }

    for (
        int index = routeLegCount;
        index > position;
        index--
    ) {
        strncpy(
            routeLegs[index],
            routeLegs[index - 1],
            sizeof(routeLegs[index]) - 1
        );

        routeLegs[index][
            sizeof(routeLegs[index]) - 1
        ] = '\0';

        routeLegSources[index] =
            routeLegSources[index - 1];

        routeLegManualIndices[index] =
            routeLegManualIndices[index - 1];

        routeLegInsertionIndices[index] =
            routeLegInsertionIndices[index - 1];
    }

    strncpy(
        routeLegs[position],
        ident,
        sizeof(routeLegs[position]) - 1
    );

    routeLegs[position][
        sizeof(routeLegs[position]) - 1
    ] = '\0';

    routeLegSources[position] =
        RouteLegSource::INSERTED;

    routeLegManualIndices[position] = -1;
    routeLegInsertionIndices[position] =
        insertionIndex;

    routeLegCount++;
}

void rebuildRouteLegs()
{
    routeLegCount = 0;

    for (int index = 0; index < MAX_ROUTE_LEGS; index++) {
        routeLegs[index][0] = '\0';
        routeLegManualIndices[index] = -1;
        routeLegInsertionIndices[index] = -1;
    }

    for (
        int index = 0;
        index < selectedDepartureLegCount;
        index++
    ) {
        if (selectedDepartureLegEnabled[index]) {
            appendComputedRouteLeg(
                selectedDepartureLegs[index],
                RouteLegSource::SD_DEPARTURE,
                index
            );
        }
    }

    for (
        int index = 0;
        index < manualRouteLegCount;
        index++
    ) {
        appendComputedRouteLeg(
            manualRouteLegs[index],
            RouteLegSource::MANUAL,
            index
        );
    }

    for (
        int index = 0;
        index < selectedArrivalLegCount;
        index++
    ) {
        if (selectedArrivalLegEnabled[index]) {
            appendComputedRouteLeg(
                selectedArrivalLegs[index],
                RouteLegSource::SD_ARRIVAL,
                index
            );
        }
    }

    // Apply LEGS-page insertions after the base route is rebuilt.
    for (
        int insertionIndex = 0;
        insertionIndex < MAX_ROUTE_INSERTIONS;
        insertionIndex++
    ) {
        if (!routeInsertions[insertionIndex].active) {
            continue;
        }

        int anchorPosition = routeLegCount;

        if (!routeInsertions[insertionIndex].beforeDestination) {
            for (
                int routeIndex = 0;
                routeIndex < routeLegCount;
                routeIndex++
            ) {
                bool indexedSource =
                    routeInsertions[insertionIndex].anchorSource == RouteLegSource::MANUAL ||
                    routeInsertions[insertionIndex].anchorSource == RouteLegSource::SD_DEPARTURE ||
                    routeInsertions[insertionIndex].anchorSource == RouteLegSource::SD_ARRIVAL;

                if (
                    routeLegSources[routeIndex] ==
                        routeInsertions[insertionIndex].anchorSource &&
                    (
                        !indexedSource ||
                        routeLegManualIndices[routeIndex] ==
                            routeInsertions[insertionIndex].anchorManualIndex
                    )
                ) {
                    anchorPosition = routeIndex;
                    break;
                }
            }
        }

        insertComputedRouteLegAt(
            anchorPosition,
            routeInsertions[insertionIndex].ident,
            insertionIndex
        );
    }

    if (activeLegIndex >= routeLegCount) {
        activeLegIndex =
            routeLegCount > 0
                ? routeLegCount - 1
                : 0;
    }

    if (routeDiscontinuityIndex > routeLegCount) {
        routeDiscontinuityIndex = routeLegCount;
    }
}

void clearDepartureSelection()
{
    selectedDepartureRunway[0] = '\0';
    selectedSid[0] = '\0';
    selectedSidTransition[0] = '\0';
    selectedDepartureLegCount = 0;

    for (int index = 0; index < MAX_PROCEDURE_LEGS; index++) {
        selectedDepartureLegs[index][0] = '\0';
        selectedDepartureLegEnabled[index] = false;
    }

    rebuildRouteLegs();
}

void clearArrivalSelection()
{
    selectedArrivalRunway[0] = '\0';
    selectedStar[0] = '\0';
    selectedStarTransition[0] = '\0';
    selectedArrivalLegCount = 0;
    arrivalTransitionCount = 0;
    starTransitionPageIndex = 0;

    for (int index = 0; index < MAX_PROCEDURE_LEGS; index++) {
        selectedArrivalLegs[index][0] = '\0';
        selectedArrivalLegEnabled[index] = false;
    }

    rebuildRouteLegs();
}


void markRouteModified()
{
    routeModified = true;
    routeExecuted = false;
    clearScratchpad();
    rebuildRouteLegs();
}


void insertRouteLegBefore(
    int routeIndex,
    const char *ident
)
{
    if (
        ident == nullptr ||
        strlen(ident) == 0
    ) {
        setScratchpad("INVALID ENTRY");
        requestPage(PageId::LEGS);
        return;
    }

    if (routeLegCount >= MAX_ROUTE_LEGS) {
        setScratchpad("ROUTE FULL");
        requestPage(PageId::LEGS);
        return;
    }

    int freeInsertionIndex = -1;

    for (
        int index = 0;
        index < MAX_ROUTE_INSERTIONS;
        index++
    ) {
        if (!routeInsertions[index].active) {
            freeInsertionIndex = index;
            break;
        }
    }

    if (freeInsertionIndex < 0) {
        setScratchpad("ROUTE FULL");
        requestPage(PageId::LEGS);
        return;
    }

    RouteInsertion &insertion =
        routeInsertions[freeInsertionIndex];

    insertion.active = true;

    strncpy(
        insertion.ident,
        ident,
        sizeof(insertion.ident) - 1
    );

    insertion.ident[
        sizeof(insertion.ident) - 1
    ] = '\0';

    insertion.beforeDestination =
        routeIndex == -1;

    if (routeIndex >= 0 && routeIndex < routeLegCount) {
        RouteLegSource selectedSource =
            routeLegSources[routeIndex];

        if (selectedSource == RouteLegSource::INSERTED) {
            int selectedInsertion =
                routeLegInsertionIndices[routeIndex];

            if (
                selectedInsertion >= 0 &&
                selectedInsertion <
                    MAX_ROUTE_INSERTIONS
            ) {
                insertion.anchorSource =
                    routeInsertions[
                        selectedInsertion
                    ].anchorSource;

                insertion.anchorManualIndex =
                    routeInsertions[
                        selectedInsertion
                    ].anchorManualIndex;

                insertion.beforeDestination =
                    routeInsertions[
                        selectedInsertion
                    ].beforeDestination;
            }
        } else {
            insertion.anchorSource = selectedSource;
            insertion.anchorManualIndex =
                routeLegManualIndices[routeIndex];
        }
    } else if (routeIndex != -1) {
        insertion.active = false;
        setScratchpad("INVALID ENTRY");
        requestPage(PageId::LEGS);
        return;
    }

    rebuildRouteLegs();

    int insertedRouteIndex = -1;

    for (
        int index = 0;
        index < routeLegCount;
        index++
    ) {
        if (
            routeLegSources[index] ==
                RouteLegSource::INSERTED &&
            routeLegInsertionIndices[index] ==
                freeInsertionIndex
        ) {
            insertedRouteIndex = index;
            break;
        }
    }

    if (pendingInsertionClosesDiscontinuity) {
        routeDiscontinuityIndex = -1;
    } else if (insertedRouteIndex >= 0) {
        routeDiscontinuityIndex =
            insertedRouteIndex + 1;
    }

    pendingInsertionClosesDiscontinuity = false;
    pendingLegInsertRouteIndex = -2;

    routeModified = true;
    routeExecuted = false;
    clearScratchpad();

    requestPage(PageId::LEGS);
}

void deleteRouteLegAt(int routeIndex)
{
    if (
        routeIndex < 0 ||
        routeIndex >= routeLegCount
    ) {
        setScratchpad("INVALID DELETE");
        requestPage(PageId::LEGS);
        return;
    }

    RouteLegSource source =
        routeLegSources[routeIndex];

    switch (source) {
        case RouteLegSource::MANUAL: {
            int manualIndex =
                routeLegManualIndices[routeIndex];

            if (
                manualIndex >= 0 &&
                manualIndex < manualRouteLegCount
            ) {
                for (
                    int index = manualIndex;
                    index < manualRouteLegCount - 1;
                    index++
                ) {
                    strncpy(
                        manualRouteLegs[index],
                        manualRouteLegs[index + 1],
                        sizeof(manualRouteLegs[index]) - 1
                    );

                    manualRouteLegs[index][
                        sizeof(manualRouteLegs[index]) - 1
                    ] = '\0';
                }

                manualRouteLegCount--;
                manualRouteLegs[
                    manualRouteLegCount
                ][0] = '\0';
            }
            break;
        }

        case RouteLegSource::SD_DEPARTURE: {
            int procedureIndex = routeLegManualIndices[routeIndex];

            if (
                procedureIndex >= 0 &&
                procedureIndex < selectedDepartureLegCount
            ) {
                selectedDepartureLegEnabled[procedureIndex] = false;
            }
            break;
        }

        case RouteLegSource::SD_ARRIVAL: {
            int procedureIndex = routeLegManualIndices[routeIndex];

            if (
                procedureIndex >= 0 &&
                procedureIndex < selectedArrivalLegCount
            ) {
                selectedArrivalLegEnabled[procedureIndex] = false;
            }
            break;
        }

        case RouteLegSource::INSERTED: {
            int insertionIndex =
                routeLegInsertionIndices[routeIndex];

            if (
                insertionIndex >= 0 &&
                insertionIndex <
                    MAX_ROUTE_INSERTIONS
            ) {
                routeInsertions[
                    insertionIndex
                ].active = false;
            }
            break;
        }

        case RouteLegSource::SID_ARD:
            omitSidArd = true;
            break;

        case RouteLegSource::SID_PTW:
            omitSidPtw = true;
            break;

        case RouteLegSource::STAR_MXE:
            omitStarMxe = true;
            break;

        case RouteLegSource::STAR_PADRZ:
            omitStarPadrz = true;
            break;

        case RouteLegSource::APPROACH:
            omitApproachLeg = true;
            break;
    }

    if (routeIndex < activeLegIndex) {
        activeLegIndex--;
    }

    if (activeLegIndex < 0) {
        activeLegIndex = 0;
    }

    rebuildRouteLegs();

    routeDiscontinuityIndex = routeIndex;

    if (routeDiscontinuityIndex > routeLegCount) {
        routeDiscontinuityIndex = routeLegCount;
    }

    routeModified = true;
    routeExecuted = false;
    clearScratchpad();

    requestPage(PageId::LEGS);
}

void legWaypointEvent(lv_event_t *event)
{
    int routeIndex =
        static_cast<int>(
            reinterpret_cast<intptr_t>(
                lv_event_get_user_data(event)
            )
        );

    if (scratchpadMessageActive) {
        return;
    }

    // -1 identifies the destination row.
    if (routeIndex == -1) {
        if (scratchpadDeleteArmed()) {
            setScratchpad("INVALID DELETE");
            requestPage(PageId::LEGS);
            return;
        }

        if (strlen(scratchpadText) > 0) {
            pendingLegInsertRouteIndex = -1;
            pendingInsertionClosesDiscontinuity = false;

            validateNavEntry(
                scratchpadText,
                InputTarget::LEGS_INSERT
            );
            return;
        }

        if (strlen(routeDestination) > 0) {
            setScratchpadEntry(routeDestination);
        }

        requestPage(PageId::LEGS);
        return;
    }

    if (
        routeIndex < 0 ||
        routeIndex >= routeLegCount
    ) {
        return;
    }

    if (scratchpadDeleteArmed()) {
        deleteRouteLegAt(routeIndex);
        return;
    }

    // With a waypoint in the scratchpad, selecting a LEGS row inserts the
    // new waypoint above the selected one and creates a discontinuity
    // between the new waypoint and the original waypoint.
    if (strlen(scratchpadText) > 0) {
        pendingLegInsertRouteIndex =
            routeIndex;

        pendingInsertionClosesDiscontinuity =
            false;

        validateNavEntry(
            scratchpadText,
            InputTarget::LEGS_INSERT
        );
        return;
    }

    // Empty scratchpad: copy the selected waypoint.
    setScratchpadEntry(
        routeLegs[routeIndex]
    );

    requestPage(PageId::LEGS);
}

void discontinuityEvent(lv_event_t *event)
{
    int discontinuityIndex =
        static_cast<int>(
            reinterpret_cast<intptr_t>(
                lv_event_get_user_data(event)
            )
        );

    if (scratchpadMessageActive) {
        return;
    }

    // DELETE + discontinuity closes/removes the discontinuity.
    if (scratchpadDeleteArmed()) {
        routeDiscontinuityIndex = -1;
        routeModified = true;
        routeExecuted = false;
        clearScratchpad();
        requestPage(PageId::LEGS);
        return;
    }

    // A waypoint selected onto the gap is inserted there and closes it.
    if (strlen(scratchpadText) > 0) {
        pendingLegInsertRouteIndex =
            discontinuityIndex >= routeLegCount
                ? -1
                : discontinuityIndex;

        pendingInsertionClosesDiscontinuity =
            true;

        validateNavEntry(
            scratchpadText,
            InputTarget::LEGS_INSERT
        );
    }
}

// =====================================================
// SD / NAV DATABASE LOOKUP
// =====================================================

void trimRight(char *textValue)
{
    if (textValue == nullptr) {
        return;
    }

    int length = static_cast<int>(strlen(textValue));

    while (
        length > 0 &&
        (
            textValue[length - 1] == ' ' ||
            textValue[length - 1] == '\r' ||
            textValue[length - 1] == '\n' ||
            textValue[length - 1] == '\t'
        )
    ) {
        textValue[length - 1] = '\0';
        length--;
    }
}

int splitPipeFields(
    char *line,
    char *fields[],
    int maximumFields
) {
    if (
        line == nullptr ||
        fields == nullptr ||
        maximumFields <= 0
    ) {
        return 0;
    }

    int count = 0;
    fields[count++] = line;

    for (
        char *cursor = line;
        *cursor != '\0' && count < maximumFields;
        cursor++
    ) {
        if (*cursor == '|') {
            *cursor = '\0';
            fields[count++] = cursor + 1;
        }
    }

    return count;
}

bool readLineAtCurrentPosition(
    File &file,
    char *buffer,
    size_t bufferSize
) {
    if (
        !file ||
        buffer == nullptr ||
        bufferSize < 2
    ) {
        return false;
    }

    size_t used = 0;

    while (file.available()) {
        int incoming = file.read();

        if (incoming < 0) {
            break;
        }

        char character = static_cast<char>(incoming);

        if (character == '\n') {
            break;
        }

        if (character == '\r') {
            continue;
        }

        if (used + 1 < bufferSize) {
            buffer[used++] = character;
        }
    }

    buffer[used] = '\0';
    return used > 0;
}

uint32_t indexRecordCount(const char *path)
{
    if (!sdMounted) {
        return 0;
    }

    File file = SD.open(path, FILE_READ);

    if (!file) {
        return 0;
    }

    uint32_t count =
        static_cast<uint32_t>(
            file.size() / SD_INDEX_RECORD_SIZE
        );

    file.close();
    return count;
}

bool findSdIndexHit(
    const char *path,
    const char *key,
    SdIndexHit &hit
) {
    if (
        !navDataReady ||
        path == nullptr ||
        key == nullptr
    ) {
        return false;
    }

    File indexFile = SD.open(path, FILE_READ);

    if (!indexFile) {
        return false;
    }

    int32_t low = 0;
    int32_t high =
        static_cast<int32_t>(
            indexFile.size() / SD_INDEX_RECORD_SIZE
        ) - 1;

    char record[SD_INDEX_RECORD_SIZE + 1];

    while (low <= high) {
        int32_t middle = low + (high - low) / 2;
        uint32_t byteOffset =
            static_cast<uint32_t>(middle) *
            SD_INDEX_RECORD_SIZE;

        if (!indexFile.seek(byteOffset)) {
            indexFile.close();
            return false;
        }

        size_t bytesRead = indexFile.readBytes(
            record,
            SD_INDEX_RECORD_SIZE
        );

        if (bytesRead != SD_INDEX_RECORD_SIZE) {
            indexFile.close();
            return false;
        }

        record[SD_INDEX_RECORD_SIZE] = '\0';

        char recordKey[9];
        memcpy(recordKey, record, 8);
        recordKey[8] = '\0';
        trimRight(recordKey);

        int comparison = strcmp(key, recordKey);

        if (comparison == 0) {
            char offsetText[11];
            char countText[7];

            memcpy(offsetText, record + 9, 10);
            offsetText[10] = '\0';

            memcpy(countText, record + 20, 6);
            countText[6] = '\0';

            hit.offset = strtoul(
                offsetText,
                nullptr,
                10
            );

            hit.count = strtoul(
                countText,
                nullptr,
                10
            );

            indexFile.close();
            return true;
        }

        if (comparison < 0) {
            high = middle - 1;
        } else {
            low = middle + 1;
        }
    }

    indexFile.close();
    return false;
}

NavType navTypeFromSdKind(char kind)
{
    switch (kind) {
        case 'V':
        case 'Q':
            return NavType::VOR;

        case 'N':
            return NavType::NDB;

        case 'W':
        case 'T':
        default:
            return NavType::FIX;
    }
}

bool parseAirportCandidate(
    char *line,
    NavCandidate &candidate
) {
    char *fields[8] = {};
    int fieldCount = splitPipeFields(
        line,
        fields,
        8
    );

    if (fieldCount < 7) {
        return false;
    }

    memset(&candidate, 0, sizeof(candidate));

    strncpy(
        candidate.ident,
        fields[0],
        sizeof(candidate.ident) - 1
    );

    candidate.type = NavType::AIRPORT;

    strncpy(
        candidate.region,
        fields[2],
        sizeof(candidate.region) - 1
    );

    candidate.latitudeE7 = strtol(
        fields[3],
        nullptr,
        10
    );

    candidate.longitudeE7 = strtol(
        fields[4],
        nullptr,
        10
    );

    strncpy(
        candidate.name,
        fields[6],
        sizeof(candidate.name) - 1
    );

    return true;
}

bool parsePointCandidate(
    char *line,
    NavCandidate &candidate
) {
    char *fields[12] = {};
    int fieldCount = splitPipeFields(
        line,
        fields,
        12
    );

    if (fieldCount < 10) {
        return false;
    }

    memset(&candidate, 0, sizeof(candidate));

    strncpy(
        candidate.ident,
        fields[0],
        sizeof(candidate.ident) - 1
    );

    candidate.type = navTypeFromSdKind(
        fields[1][0]
    );

    strncpy(
        candidate.region,
        fields[2],
        sizeof(candidate.region) - 1
    );

    strncpy(
        candidate.icao,
        fields[3],
        sizeof(candidate.icao) - 1
    );

    candidate.latitudeE7 = strtol(
        fields[4],
        nullptr,
        10
    );

    candidate.longitudeE7 = strtol(
        fields[5],
        nullptr,
        10
    );

    strncpy(
        candidate.name,
        fields[9],
        sizeof(candidate.name) - 1
    );

    return true;
}

void parseCycleFile()
{
    File cycleFile = SD.open(CYCLE_PATH, FILE_READ);

    if (!cycleFile) {
        strncpy(navStatus, "CYCLE FILE MISSING", sizeof(navStatus) - 1);
        return;
    }

    char line[96];

    while (readLineAtCurrentPosition(
        cycleFile,
        line,
        sizeof(line)
    )) {
        if (strncmp(line, "CYCLE=", 6) == 0) {
            strncpy(
                navCycle,
                line + 6,
                sizeof(navCycle) - 1
            );
        } else if (strncmp(line, "EFFECTIVE=", 10) == 0) {
            strncpy(
                navEffective,
                line + 10,
                sizeof(navEffective) - 1
            );
        } else if (strncmp(line, "EXPIRES=", 8) == 0) {
            strncpy(
                navExpires,
                line + 8,
                sizeof(navExpires) - 1
            );
        } else if (strncmp(line, "STATUS_", 7) == 0) {
            char *equals = strchr(line, '=');

            if (equals != nullptr) {
                const char *statusValue = equals + 1;

                if (strcmp(statusValue, "NEXT") == 0) {
                    strncpy(navStatus, "NEXT", sizeof(navStatus) - 1);
                } else if (strcmp(statusValue, "ACTIVE") == 0) {
                    strncpy(navStatus, "ACTIVE", sizeof(navStatus) - 1);
                } else {
                    strncpy(
                        navStatus,
                        statusValue,
                        sizeof(navStatus) - 1
                    );
                }
            }
        }
    }

    cycleFile.close();
}

bool initializeSdNavData(Board *board)
{
    if (board == nullptr) {
        strncpy(navStatus, "BOARD ERROR", sizeof(navStatus) - 1);
        return false;
    }

    // The board driver already owns and initializes the CH422G. Reuse that
    // instance instead of creating a second I2C driver, then assert EXIO4 low
    // because the TF-card chip select is active-low on this board.
    auto expander = board->getExpander();

    if (expander == nullptr) {
        strncpy(navStatus, "NO SD CONTROL", sizeof(navStatus) - 1);
        return false;
    }

    expander->digitalWrite(SD_CS_EXIO, LOW);

    SPI.setHwCs(false);
    SPI.begin(
        SD_SPI_SCK,
        SD_SPI_MISO,
        SD_SPI_MOSI,
        SD_SPI_DUMMY_CS
    );

    // Pass the dummy CS explicitly. Calling SD.begin() with no argument would
    // use ESP32-S3 GPIO10 (SS), which is also LCD B7 on this board.
    if (!SD.begin(
            SD_SPI_DUMMY_CS,
            SPI
        )) {
        strncpy(navStatus, "CARD MOUNT FAIL", sizeof(navStatus) - 1);
        return false;
    }

    sdMounted = true;

    bool allFilesPresent =
        SD.exists(AIRPORT_INDEX_PATH) &&
        SD.exists(AIRPORT_DATA_PATH) &&
        SD.exists(POINT_INDEX_PATH) &&
        SD.exists(POINT_DATA_PATH) &&
        SD.exists(PROCEDURE_INDEX_PATH) &&
        SD.exists(PROCEDURE_DATA_PATH) &&
        SD.exists(RUNWAY_INDEX_PATH) &&
        SD.exists(RUNWAY_DATA_PATH) &&
        SD.exists(CYCLE_PATH);

    if (!allFilesPresent) {
        strncpy(navStatus, "NAVDATA MISSING", sizeof(navStatus) - 1);
        return false;
    }

    navDataReady = true;
    parseCycleFile();

    // V0.9 was built on the 2608 effective date. The original package was
    // generated before activation and therefore still contains NEXT metadata.
    if (strcmp(navCycle, "2608") == 0) {
        strncpy(navStatus, "ACTIVE", sizeof(navStatus) - 1);
    }

    navAirportIndexCount = indexRecordCount(
        AIRPORT_INDEX_PATH
    );

    navPointIndexCount = indexRecordCount(
        POINT_INDEX_PATH
    );

    navProcedureAirportCount = indexRecordCount(
        PROCEDURE_INDEX_PATH
    );

    Serial.println("SD NAVDATA ONLINE");
    Serial.print("AIRAC: ");
    Serial.println(navCycle);
    Serial.print("Status: ");
    Serial.println(navStatus);
    Serial.print("Airport keys: ");
    Serial.println(navAirportIndexCount);
    Serial.print("Point keys: ");
    Serial.println(navPointIndexCount);

    return true;
}

bool navTypeAllowed(
    NavType type,
    InputTarget target
) {
    if (
        target == InputTarget::ORIGIN ||
        target == InputTarget::DESTINATION
    ) {
        return type == NavType::AIRPORT;
    }

    return true;
}

int findBuiltInNavMatches(
    const char *ident,
    InputTarget target
) {
    navMatchCount = 0;

    for (
        size_t index = 0;
        index < NAV_DATABASE_COUNT;
        index++
    ) {
        if (
            strcmp(NAV_DATABASE[index].ident, ident) == 0 &&
            navTypeAllowed(
                NAV_DATABASE[index].type,
                target
            )
        ) {
            if (navMatchCount >= MAX_MATCHES) {
                break;
            }

            NavCandidate &candidate =
                navMatches[navMatchCount];

            memset(&candidate, 0, sizeof(candidate));

            strncpy(
                candidate.ident,
                NAV_DATABASE[index].ident,
                sizeof(candidate.ident) - 1
            );

            candidate.type = NAV_DATABASE[index].type;
            strncpy(candidate.name, "BUILT-IN FALLBACK", sizeof(candidate.name) - 1);
            navMatchCount++;
        }
    }

    return navMatchCount;
}

int findSdNavMatches(
    const char *ident,
    InputTarget target
) {
    navMatchCount = 0;

    bool airportOnly =
        target == InputTarget::ORIGIN ||
        target == InputTarget::DESTINATION;

    const char *indexPath =
        airportOnly
            ? AIRPORT_INDEX_PATH
            : POINT_INDEX_PATH;

    const char *dataPath =
        airportOnly
            ? AIRPORT_DATA_PATH
            : POINT_DATA_PATH;

    SdIndexHit hit = {};

    if (!findSdIndexHit(indexPath, ident, hit)) {
        return 0;
    }

    File dataFile = SD.open(dataPath, FILE_READ);

    if (!dataFile || !dataFile.seek(hit.offset)) {
        if (dataFile) {
            dataFile.close();
        }

        return 0;
    }

    char line[256];

    for (
        uint32_t recordIndex = 0;
        recordIndex < hit.count &&
        navMatchCount < MAX_MATCHES;
        recordIndex++
    ) {
        if (!readLineAtCurrentPosition(
            dataFile,
            line,
            sizeof(line)
        )) {
            break;
        }

        bool parsed = airportOnly
            ? parseAirportCandidate(
                line,
                navMatches[navMatchCount]
            )
            : parsePointCandidate(
                line,
                navMatches[navMatchCount]
            );

        if (parsed) {
            navMatchCount++;
        }
    }

    dataFile.close();
    return navMatchCount;
}

int findNavMatches(
    const char *ident,
    InputTarget target
) {
    if (navDataReady) {
        return findSdNavMatches(ident, target);
    }

    return findBuiltInNavMatches(ident, target);
}

void applyNavCandidate(
    int candidateIndex,
    InputTarget target
) {
    if (
        candidateIndex < 0 ||
        candidateIndex >= navMatchCount
    ) {
        return;
    }

    const NavCandidate &record =
        navMatches[candidateIndex];

    switch (target) {
        case InputTarget::ORIGIN:
            strncpy(
                routeOrigin,
                record.ident,
                sizeof(routeOrigin) - 1
            );

            routeOrigin[sizeof(routeOrigin) - 1] = '\0';
            routeOriginLatitudeE7 = record.latitudeE7;
            routeOriginLongitudeE7 = record.longitudeE7;

            clearDepartureSelection();
            loadedDepartureAirport[0] = '\0';

            routeModified = true;
            routeExecuted = false;
            routeDiscontinuityIndex = -1;

            clearScratchpad();
            rebuildRouteLegs();
            break;

        case InputTarget::DESTINATION:
            strncpy(
                routeDestination,
                record.ident,
                sizeof(routeDestination) - 1
            );

            routeDestination[sizeof(routeDestination) - 1] = '\0';
            routeDestinationLatitudeE7 = record.latitudeE7;
            routeDestinationLongitudeE7 = record.longitudeE7;

            clearArrivalSelection();
            loadedArrivalAirport[0] = '\0';

            routeModified = true;
            routeExecuted = false;
            routeDiscontinuityIndex = -1;

            clearScratchpad();
            rebuildRouteLegs();
            break;

        case InputTarget::WAYPOINT:
            if (manualRouteLegCount >= MAX_ROUTE_LEGS) {
                setScratchpad("ROUTE FULL");
                return;
            }

            strncpy(
                manualRouteLegs[manualRouteLegCount],
                record.ident,
                sizeof(manualRouteLegs[manualRouteLegCount]) - 1
            );

            manualRouteLegs[manualRouteLegCount][
                sizeof(manualRouteLegs[manualRouteLegCount]) - 1
            ] = '\0';

            manualRouteLegCount++;
            routeModified = true;
            routeExecuted = false;
            rebuildRouteLegs();
            clearScratchpad();
            break;

        case InputTarget::LEGS_INSERT:
            insertRouteLegBefore(
                pendingLegInsertRouteIndex,
                record.ident
            );
            break;

        case InputTarget::DIRECT_TO:
            // The SD navdata candidates carry coordinates. The tiny built-in
            // fallback database does not, so never silently navigate to 0/0.
            if (
                !navDataReady &&
                record.latitudeE7 == 0 &&
                record.longitudeE7 == 0
            ) {
                setScratchpad("NAV POS UNAVAILABLE");
                directToTarget.valid = false;
                break;
            }

            directToTarget.valid = true;
            strncpy(
                directToTarget.ident,
                record.ident,
                sizeof(directToTarget.ident) - 1
            );
            directToTarget.ident[sizeof(directToTarget.ident) - 1] = '\0';

            strncpy(
                directToTarget.entry,
                record.ident,
                sizeof(directToTarget.entry) - 1
            );
            directToTarget.entry[sizeof(directToTarget.entry) - 1] = '\0';

            directToTarget.latitude =
                static_cast<double>(record.latitudeE7) / 10000000.0;
            directToTarget.longitude =
                static_cast<double>(record.longitudeE7) / 10000000.0;

            formatBoeingLatLon(
                directToTarget.latitude,
                directToTarget.longitude,
                directToTarget.display,
                sizeof(directToTarget.display)
            );

            clearScratchpad();
            break;

        case InputTarget::NAV_LOOKUP:
            strncpy(
                lastLookupIdent,
                record.ident,
                sizeof(lastLookupIdent) - 1
            );

            strncpy(
                lastLookupType,
                navTypeText(record.type),
                sizeof(lastLookupType) - 1
            );

            strncpy(
                lastLookupName,
                record.name,
                sizeof(lastLookupName) - 1
            );

            lastLookupLatitudeE7 = record.latitudeE7;
            lastLookupLongitudeE7 = record.longitudeE7;
            lastLookupValid = true;
            clearScratchpad();
            break;

        default:
            break;
    }
}

PageId pageForInputTarget(InputTarget target)
{
    if (target == InputTarget::NAV_LOOKUP) {
        return PageId::NAV_DATA;
    }

    if (target == InputTarget::LEGS_INSERT) {
        return PageId::LEGS;
    }

    if (target == InputTarget::DIRECT_TO) {
        return PageId::DIRECT_TO;
    }

    return PageId::ROUTE;
}

void validateNavEntry(
    const char *enteredText,
    InputTarget target
) {
    char ident[9];

    copyUppercase(
        ident,
        sizeof(ident),
        enteredText
    );

    PageId returnPage = pageForInputTarget(target);

    if (strlen(ident) == 0) {
        setScratchpad("INVALID ENTRY");
        requestPage(returnPage);
        return;
    }

    int matches = findNavMatches(ident, target);

    if (matches == 0) {
        setScratchpad("NOT IN DATABASE");

        if (target == InputTarget::NAV_LOOKUP) {
            lastLookupValid = false;
        }

        requestPage(returnPage);
        return;
    }

    if (matches == 1) {
        applyNavCandidate(0, target);
        requestPage(returnPage);
        return;
    }

    pendingMatchTarget = target;
    requestPage(PageId::SELECT_WPT);
}


// =====================================================
// SD RUNWAY / PROCEDURE READERS
// =====================================================

bool stringAlreadyStored(
    const char *value,
    char options[][9],
    int count
) {
    for (int index = 0; index < count; index++) {
        if (strcmp(options[index], value) == 0) {
            return true;
        }
    }

    return false;
}

bool runwayAlreadyStored(
    const char *value,
    char options[][8],
    int count
) {
    for (int index = 0; index < count; index++) {
        if (strcmp(options[index], value) == 0) {
            return true;
        }
    }

    return false;
}

bool loadAirportRunways(
    const char *airport,
    char options[][8],
    int maximum,
    int &count
) {
    count = 0;

    if (!navDataReady || airport == nullptr || strlen(airport) == 0) {
        return false;
    }

    SdIndexHit hit = {};

    if (!findSdIndexHit(RUNWAY_INDEX_PATH, airport, hit)) {
        return false;
    }

    File file = SD.open(RUNWAY_DATA_PATH, FILE_READ);

    if (!file || !file.seek(hit.offset)) {
        if (file) {
            file.close();
        }
        return false;
    }

    char line[220];

    for (uint32_t record = 0; record < hit.count && count < maximum; record++) {
        if (!readLineAtCurrentPosition(file, line, sizeof(line))) {
            break;
        }

        char *fields[4] = {};
        int fieldCount = splitPipeFields(line, fields, 4);

        if (fieldCount < 2 || strlen(fields[1]) == 0) {
            continue;
        }

        if (!runwayAlreadyStored(fields[1], options, count)) {
            strncpy(options[count], fields[1], 7);
            options[count][7] = '\0';
            count++;
        }
    }

    file.close();
    return count > 0;
}

bool loadProcedureOptions(
    const char *airport,
    const char *kind,
    char options[][9],
    int maximum,
    int &count
) {
    count = 0;

    if (
        !navDataReady ||
        airport == nullptr ||
        kind == nullptr ||
        strlen(airport) == 0
    ) {
        return false;
    }

    SdIndexHit hit = {};

    if (!findSdIndexHit(PROCEDURE_INDEX_PATH, airport, hit)) {
        return false;
    }

    File file = SD.open(PROCEDURE_DATA_PATH, FILE_READ);

    if (!file || !file.seek(hit.offset)) {
        if (file) {
            file.close();
        }
        return false;
    }

    char line[420];

    for (uint32_t record = 0; record < hit.count && count < maximum; record++) {
        if (!readLineAtCurrentPosition(file, line, sizeof(line))) {
            break;
        }

        char *fields[8] = {};
        int fieldCount = splitPipeFields(line, fields, 8);

        if (
            fieldCount < 3 ||
            strcmp(fields[1], kind) != 0 ||
            strlen(fields[2]) == 0
        ) {
            continue;
        }

        if (!stringAlreadyStored(fields[2], options, count)) {
            strncpy(options[count], fields[2], 8);
            options[count][8] = '\0';
            count++;
        }
    }

    file.close();
    return count > 0;
}


bool loadProcedureTransitions(
    const char *airport,
    const char *kind,
    const char *procedure,
    char options[][9],
    int maximum,
    int &count
) {
    count = 0;

    if (
        !navDataReady ||
        airport == nullptr ||
        kind == nullptr ||
        procedure == nullptr ||
        strlen(airport) == 0 ||
        strlen(procedure) == 0
    ) {
        return false;
    }

    SdIndexHit hit = {};

    if (!findSdIndexHit(PROCEDURE_INDEX_PATH, airport, hit)) {
        return false;
    }

    File file = SD.open(PROCEDURE_DATA_PATH, FILE_READ);

    if (!file || !file.seek(hit.offset)) {
        if (file) {
            file.close();
        }
        return false;
    }

    char line[420];

    for (uint32_t record = 0; record < hit.count && count < maximum; record++) {
        if (!readLineAtCurrentPosition(file, line, sizeof(line))) {
            break;
        }

        char *fields[8] = {};
        int fieldCount = splitPipeFields(line, fields, 8);

        if (
            fieldCount < 5 ||
            strcmp(fields[1], kind) != 0 ||
            strcmp(fields[2], procedure) != 0
        ) {
            continue;
        }

        const char *routeType = fields[3];
        const char *transition = fields[4];

        if (
            strlen(transition) == 0 ||
            strncmp(transition, "RW", 2) == 0 ||
            strcmp(transition, "ALL") == 0
        ) {
            continue;
        }

        bool isSelectable =
            (strcmp(kind, "SID") == 0 && strcmp(routeType, "6") == 0) ||
            (strcmp(kind, "STAR") == 0 &&
                (strcmp(routeType, "4") == 0 || strcmp(routeType, "1") == 0));

        if (
            isSelectable &&
            !stringAlreadyStored(transition, options, count)
        ) {
            strncpy(options[count], transition, 8);
            options[count][8] = '\0';
            count++;
        }
    }

    file.close();
    return count > 0;
}

bool runwayTransitionMatches(
    const char *transition,
    const char *selectedRunway
) {
    if (
        transition == nullptr ||
        selectedRunway == nullptr ||
        strlen(transition) == 0 ||
        strlen(selectedRunway) == 0
    ) {
        return false;
    }

    if (strcmp(transition, selectedRunway) == 0) {
        return true;
    }

    // ARINC runway groups such as RW09B apply to both left/right runways.
    size_t transitionLength = strlen(transition);

    if (
        transitionLength >= 5 &&
        transition[transitionLength - 1] == 'B'
    ) {
        char prefix[8] = {};
        strncpy(prefix, transition, transitionLength - 1);
        prefix[transitionLength - 1] = '\0';

        return strncmp(
            selectedRunway,
            prefix,
            strlen(prefix)
        ) == 0;
    }

    return false;
}

bool appendProcedureLeg(
    const char *fix,
    char legs[][8],
    bool enabled[],
    int maximum,
    int &count
) {
    if (fix == nullptr || strlen(fix) == 0 || count >= maximum) {
        return false;
    }

    // Avoid duplicated boundary fixes when transitions join the common route.
    if (count > 0 && strcmp(legs[count - 1], fix) == 0) {
        return true;
    }

    strncpy(legs[count], fix, 7);
    legs[count][7] = '\0';
    enabled[count] = true;
    count++;
    return true;
}

void findAutomaticProcedureTransition(
    const char *airport,
    const char *kind,
    const char *procedure,
    char *output,
    size_t outputSize
) {
    output[0] = '\0';

    SdIndexHit hit = {};

    if (!findSdIndexHit(PROCEDURE_INDEX_PATH, airport, hit)) {
        return;
    }

    File file = SD.open(PROCEDURE_DATA_PATH, FILE_READ);

    if (!file || !file.seek(hit.offset)) {
        if (file) {
            file.close();
        }
        return;
    }

    char line[420];

    for (uint32_t record = 0; record < hit.count; record++) {
        if (!readLineAtCurrentPosition(file, line, sizeof(line))) {
            break;
        }

        char *fields[8] = {};
        int fieldCount = splitPipeFields(line, fields, 8);

        if (
            fieldCount < 5 ||
            strcmp(fields[1], kind) != 0 ||
            strcmp(fields[2], procedure) != 0 ||
            strlen(fields[4]) == 0 ||
            strncmp(fields[4], "RW", 2) == 0 ||
            strcmp(fields[4], "ALL") == 0
        ) {
            continue;
        }

        bool suitable =
            (strcmp(kind, "SID") == 0 && strcmp(fields[3], "6") == 0) ||
            (strcmp(kind, "STAR") == 0 &&
                (strcmp(fields[3], "4") == 0 || strcmp(fields[3], "1") == 0));

        if (suitable) {
            strncpy(output, fields[4], outputSize - 1);
            output[outputSize - 1] = '\0';
            break;
        }
    }

    file.close();
}

bool loadSelectedProcedure(
    const char *airport,
    const char *kind,
    const char *procedure,
    const char *runway,
    const char *selectedTransition,
    char legs[][8],
    bool enabled[],
    int maximum,
    int &count
) {
    count = 0;

    for (int index = 0; index < maximum; index++) {
        legs[index][0] = '\0';
        enabled[index] = false;
    }

    if (
        !navDataReady ||
        strlen(airport) == 0 ||
        strlen(procedure) == 0
    ) {
        return false;
    }

    SdIndexHit hit = {};

    if (!findSdIndexHit(PROCEDURE_INDEX_PATH, airport, hit)) {
        return false;
    }

    File file = SD.open(PROCEDURE_DATA_PATH, FILE_READ);

    if (!file || !file.seek(hit.offset)) {
        if (file) {
            file.close();
        }
        return false;
    }

    char line[420];

    for (uint32_t record = 0; record < hit.count && count < maximum; record++) {
        if (!readLineAtCurrentPosition(file, line, sizeof(line))) {
            break;
        }

        char *fields[10] = {};
        int fieldCount = splitPipeFields(line, fields, 10);

        if (
            fieldCount < 7 ||
            strcmp(fields[1], kind) != 0 ||
            strcmp(fields[2], procedure) != 0 ||
            strlen(fields[6]) == 0
        ) {
            continue;
        }

        const char *routeType = fields[3];
        const char *transition = fields[4];
        bool include = false;

        if (strcmp(kind, "SID") == 0) {
            include =
                ((strcmp(routeType, "4") == 0 || strcmp(routeType, "T") == 0) &&
                    runwayTransitionMatches(transition, runway)) ||
                strcmp(routeType, "5") == 0 ||
                strcmp(routeType, "2") == 0 ||
                (
                    strcmp(routeType, "6") == 0 &&
                    selectedTransition != nullptr &&
                    strlen(selectedTransition) > 0 &&
                    strcmp(transition, selectedTransition) == 0
                );
        } else {
            include =
                (
                    (strcmp(routeType, "4") == 0 || strcmp(routeType, "1") == 0) &&
                    selectedTransition != nullptr &&
                    strlen(selectedTransition) > 0 &&
                    strcmp(transition, selectedTransition) == 0
                ) ||
                strcmp(routeType, "5") == 0 ||
                strcmp(routeType, "2") == 0 ||
                (
                    strcmp(routeType, "6") == 0 &&
                    runwayTransitionMatches(transition, runway)
                );
        }

        if (include) {
            appendProcedureLeg(
                fields[6],
                legs,
                enabled,
                maximum,
                count
            );
        }
    }

    file.close();
    return count > 0;
}


void refreshSelectedProcedureLegs()
{
    loadSelectedProcedure(
        routeOrigin,
        "SID",
        selectedSid,
        selectedDepartureRunway,
        selectedSidTransition,
        selectedDepartureLegs,
        selectedDepartureLegEnabled,
        MAX_PROCEDURE_LEGS,
        selectedDepartureLegCount
    );

    loadSelectedProcedure(
        routeDestination,
        "STAR",
        selectedStar,
        selectedArrivalRunway,
        selectedStarTransition,
        selectedArrivalLegs,
        selectedArrivalLegEnabled,
        MAX_PROCEDURE_LEGS,
        selectedArrivalLegCount
    );

    rebuildRouteLegs();
}




// =====================================================
// LIVE SENSOR COMPUTER / ESP-NOW
// Packet format produced by Feather_Sensor_Computer_ESPNOW_v1.ino:
// @AV1,...*CRC16\r\n
// =====================================================

uint16_t sensorCrc16Ccitt(const char *value)
{
    uint16_t crc = 0xFFFF;

    while (*value != '\0') {
        crc ^= static_cast<uint16_t>(
            static_cast<uint8_t>(*value++)
        ) << 8;

        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
            } else {
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
    }

    return crc;
}

bool sensorComputerOnline()
{
    return (
        sensorEspNowReady &&
        lastSensorPacketMs != 0 &&
        millis() - lastSensorPacketMs <= SENSOR_PACKET_TIMEOUT_MS
    );
}

bool liveGpsPositionAvailable()
{
    return (
        sensorComputerOnline() &&
        liveSensors.gpsOnline &&
        liveSensors.gpsFix
    );
}

void formatLiveGpsCompact(
    double latitude,
    double longitude,
    char *output,
    size_t outputSize,
    bool includeSpace = false
) {
    const char latitudeHemisphere = latitude >= 0.0 ? 'N' : 'S';
    const char longitudeHemisphere = longitude >= 0.0 ? 'E' : 'W';

    const double absoluteLatitude = fabs(latitude);
    const double absoluteLongitude = fabs(longitude);

    int latitudeDegrees = static_cast<int>(absoluteLatitude);
    int longitudeDegrees = static_cast<int>(absoluteLongitude);

    double latitudeMinutes =
        (absoluteLatitude - latitudeDegrees) * 60.0;
    double longitudeMinutes =
        (absoluteLongitude - longitudeDegrees) * 60.0;

    if (latitudeMinutes >= 59.9995) {
        latitudeDegrees++;
        latitudeMinutes = 0.0;
    }

    if (longitudeMinutes >= 59.9995) {
        longitudeDegrees++;
        longitudeMinutes = 0.0;
    }

    snprintf(
        output,
        outputSize,
        includeSpace
            ? "%c%02d%06.3f %c%03d%06.3f"
            : "%c%02d%06.3f%c%03d%06.3f",
        latitudeHemisphere,
        latitudeDegrees,
        latitudeMinutes,
        longitudeHemisphere,
        longitudeDegrees,
        longitudeMinutes
    );
}

bool parseLiveSensorPayload(char *payload, LiveSensorData &parsed)
{
    char *fields[24] = {};
    int fieldCount = 0;
    char *savePointer = nullptr;

    char *token = strtok_r(payload, ",", &savePointer);

    while (token != nullptr && fieldCount < 24) {
        fields[fieldCount++] = token;
        token = strtok_r(nullptr, ",", &savePointer);
    }

    if (fieldCount != 24 || strcmp(fields[0], "AV1") != 0) {
        return false;
    }

    parsed.sequence = strtoul(fields[1], nullptr, 10);
    parsed.featherUptimeMs = strtoul(fields[2], nullptr, 10);

    parsed.gpsOnline = atoi(fields[3]) != 0;
    parsed.gpsFix = atoi(fields[4]) != 0;
    parsed.satellites = static_cast<uint8_t>(atoi(fields[5]));
    parsed.latitude = atof(fields[6]);
    parsed.longitude = atof(fields[7]);
    parsed.groundSpeedKt = atof(fields[8]);
    parsed.trackDeg = atof(fields[9]);
    parsed.gpsAltitudeM = atof(fields[10]);

    parsed.utcValid = atoi(fields[11]) != 0;
    parsed.utcHour = static_cast<uint8_t>(atoi(fields[12]));
    parsed.utcMinute = static_cast<uint8_t>(atoi(fields[13]));
    parsed.utcSecond = static_cast<uint8_t>(atoi(fields[14]));

    parsed.bmeOnline = atoi(fields[15]) != 0;
    parsed.pressureHpa = atof(fields[16]);
    parsed.temperatureC = atof(fields[17]);
    parsed.humidityPercent = atof(fields[18]);
    parsed.baroAltitudeM = atof(fields[19]);

    parsed.bnoOnline = atoi(fields[20]) != 0;
    parsed.rollDeg = atof(fields[21]);
    parsed.pitchDeg = atof(fields[22]);
    parsed.headingDeg = atof(fields[23]);

    if (
        parsed.gpsFix &&
        (
            parsed.latitude < -90.0 ||
            parsed.latitude > 90.0 ||
            parsed.longitude < -180.0 ||
            parsed.longitude > 180.0
        )
    ) {
        return false;
    }

    return true;
}

bool processSensorPacket(char *line)
{
    if (line[0] != '@') {
        return false;
    }

    char *star = strrchr(line, '*');

    if (star == nullptr || strlen(star + 1) < 4) {
        return false;
    }

    const uint16_t receivedCrc = static_cast<uint16_t>(
        strtoul(star + 1, nullptr, 16)
    );

    *star = '\0';

    const char *payloadForCrc = line + 1;
    const uint16_t calculatedCrc = sensorCrc16Ccitt(payloadForCrc);

    if (receivedCrc != calculatedCrc) {
        return false;
    }

    char payloadCopy[360];

    strncpy(
        payloadCopy,
        payloadForCrc,
        sizeof(payloadCopy) - 1
    );
    payloadCopy[sizeof(payloadCopy) - 1] = '\0';

    LiveSensorData candidate = {};

    if (!parseLiveSensorPayload(payloadCopy, candidate)) {
        return false;
    }

    liveSensors = candidate;
    lastSensorPacketMs = millis();
    validSensorPacketCount++;

    return true;
}

void onSensorEspNowReceive(
    const esp_now_recv_info_t *info,
    const uint8_t *data,
    int len
) {
    if (
        data == nullptr ||
        len <= 0 ||
        len >= static_cast<int>(sizeof(sensorRxBuffer))
    ) {
        return;
    }

    // ESP-NOW callbacks run in the Wi-Fi task. Only copy here;
    // parsing and UI work stay in the normal loop.
    portENTER_CRITICAL(&sensorEspNowMux);

    memcpy(sensorRxBuffer, data, len);
    sensorRxLength = static_cast<size_t>(len);
    sensorRxBuffer[sensorRxLength] = '\0';
    sensorEspNowFrameReady = true;

    portEXIT_CRITICAL(&sensorEspNowMux);
}

bool setupSensorEspNow()
{
    WiFi.mode(WIFI_STA);
    delay(100);

    if (
        esp_wifi_set_channel(
            SENSOR_ESPNOW_CHANNEL,
            WIFI_SECOND_CHAN_NONE
        ) != ESP_OK
    ) {
        return false;
    }

    if (esp_now_init() != ESP_OK) {
        return false;
    }

    if (
        esp_now_register_recv_cb(
            onSensorEspNowReceive
        ) != ESP_OK
    ) {
        return false;
    }

    return true;
}

void serviceSensorBus()
{
    char localPacket[384];
    size_t localLength = 0;
    bool havePacket = false;

    portENTER_CRITICAL(&sensorEspNowMux);

    if (sensorEspNowFrameReady) {
        localLength = sensorRxLength;

        memcpy(
            localPacket,
            sensorRxBuffer,
            localLength + 1
        );

        sensorEspNowFrameReady = false;
        havePacket = true;
    }

    portEXIT_CRITICAL(&sensorEspNowMux);

    if (!havePacket || localLength == 0) {
        return;
    }

    if (!processSensorPacket(localPacket)) {
        badSensorPacketCount++;
    }
}


bool sensorDynamicPage(PageId page)
{
    return (
        page == PageId::POS_INIT ||
        page == PageId::DIRECT_TO ||
        page == PageId::PROGRESS ||
        page == PageId::LAT_LON ||
        page == PageId::STATUS
    );
}

void serviceSensorPageRefresh()
{
    if (
        redrawRequested ||
        keyboardOverlay != nullptr ||
        !sensorDynamicPage(currentPage) ||
        millis() - lastSensorPageRefreshMs < SENSOR_PAGE_REFRESH_MS
    ) {
        return;
    }

    lastSensorPageRefreshMs = millis();
    requestPage(currentPage);
}

// =====================================================
// BOEING-STYLE LAT/LON PARSER / NAV MATH
// =====================================================

void formatBoeingLatLon(
    double latitude,
    double longitude,
    char *output,
    size_t size
) {
    char latitudeHemisphere = latitude >= 0.0 ? 'N' : 'S';
    char longitudeHemisphere = longitude >= 0.0 ? 'E' : 'W';

    double absoluteLatitude = fabs(latitude);
    double absoluteLongitude = fabs(longitude);

    int latitudeDegrees = static_cast<int>(absoluteLatitude);
    int longitudeDegrees = static_cast<int>(absoluteLongitude);

    double latitudeMinutes =
        (absoluteLatitude - latitudeDegrees) * 60.0;
    double longitudeMinutes =
        (absoluteLongitude - longitudeDegrees) * 60.0;

    snprintf(
        output,
        size,
        "%c%02d %04.1f %c%03d %04.1f",
        latitudeHemisphere,
        latitudeDegrees,
        latitudeMinutes,
        longitudeHemisphere,
        longitudeDegrees,
        longitudeMinutes
    );
}

bool parseBoeingLatLon(
    const char *input,
    GeoPosition &position
) {
    char compact[32] = {};
    int writeIndex = 0;

    for (int index = 0; input[index] != '\0' && writeIndex < 31; index++) {
        char value = input[index];

        if (value >= 'a' && value <= 'z') {
            value = static_cast<char>(value - 'a' + 'A');
        }

        if (
            value == ' ' ||
            value == '/' ||
            value == ',' ||
            value == '\t'
        ) {
            continue;
        }

        compact[writeIndex++] = value;
    }

    compact[writeIndex] = '\0';

    if (writeIndex < 15 || (compact[0] != 'N' && compact[0] != 'S')) {
        return false;
    }

    int longitudeHemisphereIndex = -1;

    for (int index = 1; index < writeIndex; index++) {
        if (compact[index] == 'E' || compact[index] == 'W') {
            longitudeHemisphereIndex = index;
            break;
        }
    }

    if (longitudeHemisphereIndex < 0) {
        return false;
    }

    char latitudeDigits[12] = {};
    char longitudeDigits[12] = {};

    int latitudeLength = longitudeHemisphereIndex - 1;
    int longitudeLength = writeIndex - longitudeHemisphereIndex - 1;

    if (
        latitudeLength < 5 || latitudeLength > 8 ||
        longitudeLength < 6 || longitudeLength > 9
    ) {
        return false;
    }

    strncpy(latitudeDigits, compact + 1, latitudeLength);
    strncpy(
        longitudeDigits,
        compact + longitudeHemisphereIndex + 1,
        longitudeLength
    );

    char latitudeDegreeText[3] = {
        latitudeDigits[0],
        latitudeDigits[1],
        '\0'
    };

    char longitudeDegreeText[4] = {
        longitudeDigits[0],
        longitudeDigits[1],
        longitudeDigits[2],
        '\0'
    };

    double latitudeDegrees = atof(latitudeDegreeText);
    double longitudeDegrees = atof(longitudeDegreeText);
    double latitudeMinutes = atof(latitudeDigits + 2);
    double longitudeMinutes = atof(longitudeDigits + 3);

    if (
        latitudeDegrees > 90.0 ||
        longitudeDegrees > 180.0 ||
        latitudeMinutes >= 60.0 ||
        longitudeMinutes >= 60.0
    ) {
        return false;
    }

    double latitude = latitudeDegrees + latitudeMinutes / 60.0;
    double longitude = longitudeDegrees + longitudeMinutes / 60.0;

    if (compact[0] == 'S') {
        latitude = -latitude;
    }

    if (compact[longitudeHemisphereIndex] == 'W') {
        longitude = -longitude;
    }

    position.valid = true;
    position.latitude = latitude;
    position.longitude = longitude;

    snprintf(
        position.entry,
        sizeof(position.entry),
        "%c%02d%04.1f%c%03d%04.1f",
        compact[0],
        static_cast<int>(latitudeDegrees),
        latitudeMinutes,
        compact[longitudeHemisphereIndex],
        static_cast<int>(longitudeDegrees),
        longitudeMinutes
    );

    formatBoeingLatLon(
        latitude,
        longitude,
        position.display,
        sizeof(position.display)
    );

    return true;
}

double degreesToRadians(double degrees)
{
    return degrees * 3.14159265358979323846 / 180.0;
}

double greatCircleDistanceNm(
    double lat1,
    double lon1,
    double lat2,
    double lon2
) {
    double latitude1 = degreesToRadians(lat1);
    double latitude2 = degreesToRadians(lat2);
    double deltaLatitude = degreesToRadians(lat2 - lat1);
    double deltaLongitude = degreesToRadians(lon2 - lon1);

    double a =
        sin(deltaLatitude / 2.0) * sin(deltaLatitude / 2.0) +
        cos(latitude1) * cos(latitude2) *
        sin(deltaLongitude / 2.0) * sin(deltaLongitude / 2.0);

    double centralAngle = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    return 3440.065 * centralAngle;
}

double initialTrueCourse(
    double lat1,
    double lon1,
    double lat2,
    double lon2
) {
    double latitude1 = degreesToRadians(lat1);
    double latitude2 = degreesToRadians(lat2);
    double deltaLongitude = degreesToRadians(lon2 - lon1);

    double y = sin(deltaLongitude) * cos(latitude2);
    double x =
        cos(latitude1) * sin(latitude2) -
        sin(latitude1) * cos(latitude2) * cos(deltaLongitude);

    double bearing = atan2(y, x) * 180.0 / 3.14159265358979323846;

    while (bearing < 0.0) {
        bearing += 360.0;
    }

    while (bearing >= 360.0) {
        bearing -= 360.0;
    }

    return bearing;
}

// =====================================================
// BASIC UI HELPERS
// =====================================================

lv_obj_t *createLabel(
    lv_obj_t *parent,
    const char *text,
    int x,
    int y,
    lv_color_t color = CDU_WHITE,
    const lv_font_t *font =
        &lv_font_montserrat_16
) {
    lv_obj_t *label =
        lv_label_create(parent);

    lv_label_set_text(label, text);

    lv_obj_set_style_text_color(
        label,
        color,
        0
    );

    lv_obj_set_style_text_font(
        label,
        font,
        0
    );

    lv_obj_set_style_text_line_space(
        label,
        4,
        0
    );

    lv_obj_align(
        label,
        LV_ALIGN_TOP_LEFT,
        x,
        y
    );

    return label;
}

void createLine(
    lv_obj_t *parent,
    int x,
    int y,
    int width,
    int height,
    lv_color_t color = CDU_WHITE
) {
    lv_obj_t *line =
        lv_obj_create(parent);

    lv_obj_set_size(
        line,
        width,
        height
    );

    lv_obj_align(
        line,
        LV_ALIGN_TOP_LEFT,
        x,
        y
    );

    lv_obj_set_style_bg_color(
        line,
        color,
        0
    );

    lv_obj_set_style_bg_opa(
        line,
        LV_OPA_COVER,
        0
    );

    lv_obj_set_style_border_width(
        line,
        0,
        0
    );

    lv_obj_set_style_radius(
        line,
        0,
        0
    );

    lv_obj_clear_flag(
        line,
        LV_OBJ_FLAG_SCROLLABLE
    );
}

void pageButtonEvent(lv_event_t *event)
{
    PageId page =
        static_cast<PageId>(
            reinterpret_cast<intptr_t>(
                lv_event_get_user_data(event)
            )
        );

    requestPage(page);
}

lv_obj_t *createButton(
    lv_obj_t *parent,
    const char *text,
    int x,
    int y,
    int width,
    int height,
    lv_event_cb_t callback,
    void *userData,
    lv_color_t textColor = CDU_WHITE,
    lv_color_t borderColor = CDU_WHITE,
    const lv_font_t *font = &lv_font_montserrat_16
) {
    lv_obj_t *button =
        lv_btn_create(parent);

    lv_obj_set_size(
        button,
        width,
        height
    );

    lv_obj_align(
        button,
        LV_ALIGN_TOP_LEFT,
        x,
        y
    );

    lv_obj_set_style_bg_color(
        button,
        CDU_BLACK,
        LV_PART_MAIN |
            LV_STATE_DEFAULT
    );

    lv_obj_set_style_bg_color(
        button,
        CDU_DARK_GRAY,
        LV_PART_MAIN |
            LV_STATE_PRESSED
    );

    lv_obj_set_style_border_color(
        button,
        borderColor,
        LV_PART_MAIN
    );

    lv_obj_set_style_border_width(
        button,
        1,
        LV_PART_MAIN
    );

    lv_obj_set_style_radius(
        button,
        0,
        LV_PART_MAIN
    );

    lv_obj_add_event_cb(
        button,
        callback,
        LV_EVENT_CLICKED,
        userData
    );

    lv_obj_t *label =
        lv_label_create(button);

    lv_label_set_text(label, text);

    lv_obj_set_style_text_color(
        label,
        textColor,
        0
    );

    lv_obj_set_style_text_font(
        label,
        font,
        0
    );

    lv_obj_center(label);

    return button;
}

lv_obj_t *createPageButton(
    lv_obj_t *parent,
    const char *text,
    int x,
    int y,
    int width,
    int height,
    PageId destination
) {
    return createButton(
        parent,
        text,
        x,
        y,
        width,
        height,
        pageButtonEvent,
        reinterpret_cast<void *>(
            static_cast<intptr_t>(
                destination
            )
        )
    );
}

// Legacy development selector retained only so the old helper links.
// V0.9 uses dynamicProcedureSelectionEvent() for SD procedures.
void procedureSelectionEvent(lv_event_t *event)
{
    (void)event;
}

lv_obj_t *createSelectionButton(
    lv_obj_t *parent,
    const char *text,
    int x,
    int y,
    int width,
    int height,
    ProcedureAction action,
    bool selected
) {
    lv_obj_t *button = createButton(
        parent,
        text,
        x,
        y,
        width,
        height,
        procedureSelectionEvent,
        reinterpret_cast<void *>(
            static_cast<intptr_t>(action)
        ),
        selected ? CDU_BLACK : CDU_WHITE,
        selected ? CDU_REVERSE_VIDEO : CDU_MEDIUM_GRAY
    );

    if (selected) {
        lv_obj_set_style_bg_color(
            button,
            CDU_REVERSE_VIDEO,
            LV_PART_MAIN | LV_STATE_DEFAULT
        );
    }

    return button;
}

lv_obj_t *createExecButton(
    lv_obj_t *parent,
    int x,
    int y,
    PageId returnPage
) {
    lv_obj_t *button = createButton(
        parent,
        "EXEC",
        x,
        y,
        110,
        45,
        execEvent,
        reinterpret_cast<void *>(
            static_cast<intptr_t>(returnPage)
        ),
        routeModified ? CDU_BLACK : CDU_MEDIUM_GRAY,
        routeModified ? CDU_WHITE : CDU_MEDIUM_GRAY
    );

    lv_obj_set_style_bg_color(
        button,
        routeModified ? CDU_WHITE : CDU_BLACK,
        LV_PART_MAIN | LV_STATE_DEFAULT
    );

    return button;
}

void prepareScreen(
    const char *title,
    const char *pageCounter = ""
) {
    lv_obj_t *screen =
        lv_scr_act();

    lv_obj_clean(screen);

    lv_obj_set_style_bg_color(
        screen,
        CDU_BLACK,
        0
    );

    lv_obj_set_style_bg_opa(
        screen,
        LV_OPA_COVER,
        0
    );

    lv_obj_clear_flag(
        screen,
        LV_OBJ_FLAG_SCROLLABLE
    );

    createLabel(
        screen,
        title,
        20,
        8,
        CDU_WHITE,
        &lv_font_montserrat_30
    );

    if (strlen(pageCounter) > 0) {
        createLabel(
            screen,
            pageCounter,
            730,
            18,
            CDU_WHITE
        );
    }

    createLine(
        screen,
        20,
        52,
        760,
        2,
        CDU_WHITE
    );
}

// =====================================================
// SCRATCHPAD
// =====================================================

void scratchpadEvent(lv_event_t *event)
{
    openKeyboard(
        InputTarget::SCRATCHPAD
    );
}

void createScratchpad(lv_obj_t *parent)
{
    createLine(
        parent,
        20,
        424,
        760,
        1,
        CDU_WHITE
    );

    lv_obj_t *touchArea =
        lv_btn_create(parent);

    lv_obj_set_size(
        touchArea,
        760,
        48
    );

    lv_obj_align(
        touchArea,
        LV_ALIGN_TOP_LEFT,
        20,
        426
    );

    lv_obj_set_style_bg_color(
        touchArea,
        CDU_BLACK,
        LV_PART_MAIN
    );

    lv_obj_set_style_border_width(
        touchArea,
        0,
        LV_PART_MAIN
    );

    lv_obj_set_style_radius(
        touchArea,
        0,
        LV_PART_MAIN
    );

    lv_obj_add_event_cb(
        touchArea,
        scratchpadEvent,
        LV_EVENT_CLICKED,
        nullptr
    );

    lv_obj_t *label =
        lv_label_create(touchArea);

    if (strlen(scratchpadText) == 0) {
        lv_label_set_text(
            label,
            "TAP SCRATCHPAD"
        );
    } else {
        lv_label_set_text(
            label,
            scratchpadText
        );
    }

    lv_obj_set_style_text_color(
        label,
        CDU_WHITE,
        0
    );

    lv_obj_set_style_text_font(
        label,
        &lv_font_montserrat_16,
        0
    );

    lv_obj_align(
        label,
        LV_ALIGN_LEFT_MID,
        8,
        0
    );
}

// =====================================================
// FIELD BUTTONS
// =====================================================

void inputFieldEvent(lv_event_t *event)
{
    InputTarget target =
        static_cast<InputTarget>(
            reinterpret_cast<intptr_t>(
                lv_event_get_user_data(event)
            )
        );

    openKeyboard(target);
}

lv_obj_t *createInputField(
    lv_obj_t *parent,
    const char *text,
    int x,
    int y,
    int width,
    int height,
    InputTarget target
) {
    return createButton(
        parent,
        text,
        x,
        y,
        width,
        height,
        inputFieldEvent,
        reinterpret_cast<void *>(
            static_cast<intptr_t>(
                target
            )
        ),
        CDU_WHITE,
        CDU_MEDIUM_GRAY
    );
}

// =====================================================
// KEYBOARD
// =====================================================

void keyboardActionEvent(lv_event_t *event)
{
    KeyboardAction action =
        static_cast<KeyboardAction>(
            reinterpret_cast<intptr_t>(
                lv_event_get_user_data(event)
            )
        );

    if (keyboardTextArea == nullptr) {
        return;
    }

    // FMC scratchpad messages are protected. While a message such as
    // NOT IN DATABASE is displayed, entry and ENTER/CLR do nothing.
    // DEL is the only key that clears the message.
    if (
        activeInputTarget == InputTarget::SCRATCHPAD &&
        scratchpadMessageActive
    ) {
        if (action == KeyboardAction::DELETE_CHAR) {
            clearScratchpad();
            closeKeyboard();
            requestPage(currentPage);
        } else if (action == KeyboardAction::CANCEL) {
            closeKeyboard();
        }

        return;
    }

    switch (action) {
        case KeyboardAction::CLEAR:
            lv_textarea_set_text(
                keyboardTextArea,
                ""
            );
            break;

        case KeyboardAction::DELETE_CHAR:
            // With an empty scratchpad, DEL arms the CDU DELETE command.
            if (
                activeInputTarget == InputTarget::SCRATCHPAD &&
                strlen(
                    lv_textarea_get_text(
                        keyboardTextArea
                    )
                ) == 0
            ) {
                setScratchpadEntry("DELETE");
                closeKeyboard();
                requestPage(currentPage);
                return;
            }

            lv_textarea_del_char(
                keyboardTextArea
            );
            break;

        case KeyboardAction::CANCEL:
            closeKeyboard();
            break;

        case KeyboardAction::ENTER:
            commitKeyboardInput();
            closeKeyboard();
            break;
    }
}

void keyboardEvent(lv_event_t *event)
{
    lv_event_code_t code =
        lv_event_get_code(event);

    if (
        activeInputTarget == InputTarget::SCRATCHPAD &&
        scratchpadMessageActive
    ) {
        if (code == LV_EVENT_CANCEL) {
            closeKeyboard();
        }

        return;
    }

    if (code == LV_EVENT_READY) {
        commitKeyboardInput();
        closeKeyboard();
    }

    if (code == LV_EVENT_CANCEL) {
        closeKeyboard();
    }
}

void openKeyboard(InputTarget target)
{
    if (keyboardOverlay != nullptr) {
        return;
    }

    // While an FMC message is present, all normal entry is inhibited.
    // Any field tap opens the protected message; DEL must clear it first.
    if (scratchpadMessageActive) {
        target = InputTarget::SCRATCHPAD;
    }

    activeInputTarget = target;

    keyboardOverlay =
        lv_obj_create(lv_layer_top());

    lv_obj_set_size(
        keyboardOverlay,
        800,
        480
    );

    lv_obj_align(
        keyboardOverlay,
        LV_ALIGN_CENTER,
        0,
        0
    );

    lv_obj_set_style_bg_color(
        keyboardOverlay,
        CDU_BLACK,
        0
    );

    lv_obj_set_style_bg_opa(
        keyboardOverlay,
        LV_OPA_COVER,
        0
    );

    lv_obj_set_style_border_width(
        keyboardOverlay,
        0,
        0
    );

    lv_obj_set_style_radius(
        keyboardOverlay,
        0,
        0
    );

    lv_obj_set_style_pad_all(
        keyboardOverlay,
        0,
        0
    );

    lv_obj_clear_flag(
        keyboardOverlay,
        LV_OBJ_FLAG_SCROLLABLE
    );

    const char *title =
        scratchpadMessageActive &&
        target == InputTarget::SCRATCHPAD
            ? "FMC MESSAGE - PRESS DEL"
            : "SCRATCHPAD";

    const char *initialText = scratchpadText;
    uint32_t maxLength = 24;

    switch (target) {
        case InputTarget::ORIGIN:
            title = "ENTER ORIGIN";
            initialText = routeOrigin;
            maxLength = 4;
            break;

        case InputTarget::DESTINATION:
            title = "ENTER DESTINATION";
            initialText = routeDestination;
            maxLength = 4;
            break;

        case InputTarget::WAYPOINT:
            title = "ENTER WAYPOINT";
            initialText = "";
            maxLength = 7;
            break;

        case InputTarget::NAV_LOOKUP:
            title = "NAV DATA LOOKUP";
            initialText = "";
            maxLength = 7;
            break;

        case InputTarget::LEGS_INSERT:
            title = "INSERT WAYPOINT";
            initialText = scratchpadText;
            maxLength = 7;
            break;

        case InputTarget::DIRECT_TO:
            title = "DIRECT TO WPT / LAT-LON";
            initialText = directToTarget.valid
                ? directToTarget.entry
                : "";
            maxLength = 24;
            break;

        case InputTarget::LATLON_FROM:
            title = "ENTER FROM LAT/LON";
            initialText = manualFromPosition.valid
                ? manualFromPosition.entry
                : "";
            maxLength = 22;
            break;

        case InputTarget::LATLON_TO:
            title = "ENTER TO LAT/LON";
            initialText = manualToPosition.valid
                ? manualToPosition.entry
                : "";
            maxLength = 22;
            break;

        default:
            break;
    }

    createLabel(
        keyboardOverlay,
        title,
        20,
        10,
        CDU_WHITE,
        &lv_font_montserrat_30
    );

    keyboardTextArea =
        lv_textarea_create(keyboardOverlay);

    lv_obj_set_size(
        keyboardTextArea,
        490,
        52
    );

    lv_obj_align(
        keyboardTextArea,
        LV_ALIGN_TOP_LEFT,
        20,
        58
    );

    lv_textarea_set_one_line(
        keyboardTextArea,
        true
    );

    lv_textarea_set_max_length(
        keyboardTextArea,
        maxLength
    );

    lv_textarea_set_text(
        keyboardTextArea,
        initialText
    );

    lv_textarea_set_accepted_chars(
        keyboardTextArea,
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 ./-"
    );

    lv_obj_set_style_bg_color(
        keyboardTextArea,
        CDU_BLACK,
        LV_PART_MAIN
    );

    lv_obj_set_style_border_color(
        keyboardTextArea,
        CDU_WHITE,
        LV_PART_MAIN
    );

    lv_obj_set_style_border_width(
        keyboardTextArea,
        1,
        LV_PART_MAIN
    );

    lv_obj_set_style_text_color(
        keyboardTextArea,
        CDU_WHITE,
        LV_PART_MAIN
    );

    lv_obj_set_style_text_font(
        keyboardTextArea,
        &lv_font_montserrat_30,
        LV_PART_MAIN
    );

    createButton(
        keyboardOverlay,
        "CLR",
        525,
        58,
        75,
        52,
        keyboardActionEvent,
        reinterpret_cast<void *>(
            static_cast<intptr_t>(
                KeyboardAction::CLEAR
            )
        )
    );

    createButton(
        keyboardOverlay,
        "DEL",
        610,
        58,
        75,
        52,
        keyboardActionEvent,
        reinterpret_cast<void *>(
            static_cast<intptr_t>(
                KeyboardAction::DELETE_CHAR
            )
        )
    );

    createButton(
        keyboardOverlay,
        "ENTER",
        695,
        58,
        85,
        52,
        keyboardActionEvent,
        reinterpret_cast<void *>(
            static_cast<intptr_t>(
                KeyboardAction::ENTER
            )
        )
    );

    createButton(
        keyboardOverlay,
        "CANCEL",
        650,
        118,
        130,
        44,
        keyboardActionEvent,
        reinterpret_cast<void *>(
            static_cast<intptr_t>(
                KeyboardAction::CANCEL
            )
        )
    );

    keyboardObject =
        lv_keyboard_create(keyboardOverlay);

    lv_obj_set_size(
        keyboardObject,
        800,
        300
    );

    lv_obj_align(
        keyboardObject,
        LV_ALIGN_BOTTOM_MID,
        0,
        0
    );

    lv_keyboard_set_mode(
        keyboardObject,
        LV_KEYBOARD_MODE_TEXT_UPPER
    );

    lv_keyboard_set_textarea(
        keyboardObject,
        keyboardTextArea
    );

    lv_obj_set_style_bg_color(
        keyboardObject,
        CDU_BLACK,
        LV_PART_MAIN
    );

    lv_obj_set_style_border_color(
        keyboardObject,
        CDU_WHITE,
        LV_PART_MAIN
    );

    lv_obj_set_style_border_width(
        keyboardObject,
        1,
        LV_PART_MAIN
    );

    lv_obj_set_style_bg_color(
        keyboardObject,
        CDU_DARK_GRAY,
        LV_PART_ITEMS |
            LV_STATE_DEFAULT
    );

    lv_obj_set_style_bg_color(
        keyboardObject,
        CDU_MEDIUM_GRAY,
        LV_PART_ITEMS |
            LV_STATE_PRESSED
    );

    lv_obj_set_style_text_color(
        keyboardObject,
        CDU_WHITE,
        LV_PART_ITEMS
    );

    lv_obj_add_event_cb(
        keyboardObject,
        keyboardEvent,
        LV_EVENT_ALL,
        nullptr
    );

    if (
        activeInputTarget == InputTarget::SCRATCHPAD &&
        scratchpadMessageActive
    ) {
        // Disable alpha/numeric keys while the protected FMC message is
        // present. The separate DEL and CANCEL buttons still work.
        lv_obj_add_state(
            keyboardObject,
            LV_STATE_DISABLED
        );
    }
}

void closeKeyboard()
{
    if (keyboardOverlay != nullptr) {
        lv_obj_del_async(
            keyboardOverlay
        );
    }

    keyboardOverlay = nullptr;
    keyboardTextArea = nullptr;
    keyboardObject = nullptr;

    activeInputTarget =
        InputTarget::NONE;

    keyboardClosedAtMs = millis();
}

void commitKeyboardInput()
{
    if (keyboardTextArea == nullptr) {
        return;
    }

    const char *enteredText =
        lv_textarea_get_text(
            keyboardTextArea
        );

    if (
        activeInputTarget ==
        InputTarget::SCRATCHPAD
    ) {
        if (scratchpadMessageActive) {
            return;
        }

        char entry[32];

        copyUppercase(
            entry,
            sizeof(entry),
            enteredText
        );

        setScratchpadEntry(entry);

        redrawRequested = true;
        return;
    }

    if (activeInputTarget == InputTarget::DIRECT_TO) {
        char entry[32] = {};
        copyUppercase(entry, sizeof(entry), enteredText);

        // Coordinate entries use the same Boeing-style parser as the existing
        // LAT/LON page. Otherwise treat the entry as a navdata identifier.
        bool looksLikeCoordinate =
            (entry[0] == 'N' || entry[0] == 'S') &&
            entry[1] >= '0' && entry[1] <= '9' &&
            entry[2] >= '0' && entry[2] <= '9';

        if (looksLikeCoordinate) {
            GeoPosition parsed = {};

            if (!parseBoeingLatLon(entry, parsed)) {
                setScratchpad("INVALID LAT/LON");
                requestPage(PageId::DIRECT_TO);
                return;
            }

            directToTarget.valid = true;
            strncpy(
                directToTarget.ident,
                "LAT/LON",
                sizeof(directToTarget.ident) - 1
            );
            directToTarget.ident[sizeof(directToTarget.ident) - 1] = '\0';

            strncpy(
                directToTarget.entry,
                parsed.entry,
                sizeof(directToTarget.entry) - 1
            );
            directToTarget.entry[sizeof(directToTarget.entry) - 1] = '\0';

            strncpy(
                directToTarget.display,
                parsed.display,
                sizeof(directToTarget.display) - 1
            );
            directToTarget.display[sizeof(directToTarget.display) - 1] = '\0';

            directToTarget.latitude = parsed.latitude;
            directToTarget.longitude = parsed.longitude;

            clearScratchpad();
            requestPage(PageId::DIRECT_TO);
            return;
        }

        validateNavEntry(entry, InputTarget::DIRECT_TO);
        return;
    }

    if (
        activeInputTarget == InputTarget::LATLON_FROM ||
        activeInputTarget == InputTarget::LATLON_TO
    ) {
        GeoPosition parsed = {};

        if (!parseBoeingLatLon(enteredText, parsed)) {
            setScratchpad("INVALID LAT/LON");
            requestPage(PageId::LAT_LON);
            return;
        }

        if (activeInputTarget == InputTarget::LATLON_FROM) {
            manualFromPosition = parsed;
        } else {
            manualToPosition = parsed;
        }

        clearScratchpad();
        requestPage(PageId::LAT_LON);
        return;
    }

    validateNavEntry(
        enteredText,
        activeInputTarget
    );
}
// =====================================================
// CDU LANDING PAGE
// =====================================================

void createCduMenuPage()
{
    prepareScreen(
        "CDU MENU",
        "1/1"
    );

    lv_obj_t *screen =
        lv_scr_act();

    createPageButton(
        screen,
        "<FMC",
        25,
        130,
        220,
        55,
        PageId::IDENT
    );

    createPageButton(
        screen,
        "STATUS>",
        555,
        130,
        220,
        55,
        PageId::STATUS
    );

    createLabel(
        screen,
        "SELECT DESIRED SYSTEM",
        290,
        235,
        CDU_WHITE
    );

    createLabel(
        screen,
        "MINI AVIONICS CDU\n"
        "DISPLAY UNIT ONLINE",
        305,
        285,
        CDU_WHITE
    );

    createScratchpad(screen);
}
// =====================================================
// FMC IDENT PAGE
// =====================================================

void createIdentPage()
{
    prepareScreen(
        "IDENT",
        "1/1"
    );

    lv_obj_t *screen = lv_scr_act();

    createLabel(screen, "MODEL", 35, 72, CDU_WHITE);
    createLabel(
        screen,
        "IN AIRAC 2608",
        35,
        98,
        CDU_WHITE,
        &lv_font_montserrat_30
    );

    createLabel(screen, "ENG RATING", 35, 150, CDU_WHITE);
    createLabel(
        screen,
        "N/A",
        35,
        176,
        CDU_WHITE,
        &lv_font_montserrat_30
    );

    char navDataText[40];
    snprintf(
        navDataText,
        sizeof(navDataText),
        navDataReady
            ? "FAA %s + WORLD"
            : "FALLBACK TEST",
        navCycle
    );

    createLabel(screen, "NAV DATA", 470, 72, CDU_WHITE);
    createLabel(
        screen,
        navDataText,
        470,
        98,
        CDU_WHITE,
        &lv_font_montserrat_30
    );

    createLabel(screen, "STATUS", 470, 150, CDU_WHITE);
    createLabel(
        screen,
        navDataReady ? navStatus : "SD NOT READY",
        470,
        176,
        CDU_WHITE,
        &lv_font_montserrat_30
    );

    createLabel(screen, "OP PROGRAM", 35, 250, CDU_WHITE);
    createLabel(
        screen,
        "MINI FMC V0.12",
        35,
        276,
        CDU_WHITE,
        &lv_font_montserrat_30
    );

    createPageButton(
        screen,
        "<INDEX",
        20,
        365,
        145,
        45,
        PageId::INDEX
    );

    createPageButton(
        screen,
        "POS INIT>",
        620,
        365,
        160,
        45,
        PageId::POS_INIT
    );

    createScratchpad(screen);
}
// =====================================================
// POSITION INITIALIZATION ACTIONS
// =====================================================

void lastPositionEvent(lv_event_t *event)
{
    char lastPosition[32];

    snprintf(
        lastPosition,
        sizeof(lastPosition),
        "%s %s",
        LAST_POSITION_LAT,
        LAST_POSITION_LON
    );

    setScratchpadEntry(lastPosition);

    requestPage(PageId::POS_INIT);
}

void setIrsPositionEvent(lv_event_t *event)
{
    if (strlen(scratchpadText) == 0) {
        setScratchpad("SELECT POSITION");
        requestPage(PageId::POS_INIT);
        return;
    }

    GeoPosition parsedPosition = {};

    if (!parseBoeingLatLon(scratchpadText, parsedPosition)) {
        setScratchpad("INVALID IRS POS");
        requestPage(PageId::POS_INIT);
        return;
    }

    strncpy(
        irsPosition,
        scratchpadText,
        sizeof(irsPosition) - 1
    );

    irsPosition[sizeof(irsPosition) - 1] = '\0';

    irsPositionSet = true;
    clearScratchpad();

    requestPage(PageId::POS_INIT);
}



void gpsPositionEvent(lv_event_t *event)
{
    (void)event;

    if (!liveGpsPositionAvailable()) {
        setScratchpad("GPS POSITION NOT AVAILABLE");
        requestPage(PageId::POS_INIT);
        return;
    }

    char gpsEntry[32];

    formatLiveGpsCompact(
        liveSensors.latitude,
        liveSensors.longitude,
        gpsEntry,
        sizeof(gpsEntry),
        false
    );

    setScratchpadEntry(gpsEntry);
    requestPage(PageId::POS_INIT);
}

// =====================================================
// POSITION INITIALIZATION PAGE
// =====================================================

void createPosInitPage()
{
    prepareScreen("POS INIT", "1/3");
    lv_obj_t *screen = lv_scr_act();

    createLabel(screen, "LAST POS", 35, 70, CDU_WHITE);

    char lastPositionText[96];

    snprintf(
        lastPositionText,
        sizeof(lastPositionText),
        "%s  %s",
        LAST_POSITION_LAT,
        LAST_POSITION_LON
    );

    createButton(
        screen,
        lastPositionText,
        25,
        95,
        365,
        52,
        lastPositionEvent,
        nullptr,
        CDU_WHITE,
        CDU_MEDIUM_GRAY,
        &lv_font_montserrat_30
    );

    createLabel(screen, "REF AIRPORT", 35, 165, CDU_WHITE);
    createLabel(
        screen,
        LAST_POSITION_AIRPORT,
        35,
        192,
        CDU_WHITE,
        &lv_font_montserrat_30
    );

    createLabel(screen, "GATE", 35, 250, CDU_WHITE);
    createLabel(
        screen,
        "----",
        35,
        277,
        CDU_WHITE,
        &lv_font_montserrat_30
    );

    createLabel(screen, "UTC", 550, 70, CDU_WHITE);

    char utcText[20] = "--:--Z";

    if (
        sensorComputerOnline() &&
        liveSensors.utcValid
    ) {
        snprintf(
            utcText,
            sizeof(utcText),
            "%02u:%02u:%02uZ",
            liveSensors.utcHour,
            liveSensors.utcMinute,
            liveSensors.utcSecond
        );
    }

    createLabel(
        screen,
        utcText,
        515,
        97,
        CDU_WHITE,
        &lv_font_montserrat_30
    );

    createLabel(screen, "GPS POS", 500, 155, CDU_WHITE);

    if (liveGpsPositionAvailable()) {
        char gpsDisplay[40];

        formatLiveGpsCompact(
            liveSensors.latitude,
            liveSensors.longitude,
            gpsDisplay,
            sizeof(gpsDisplay),
            true
        );

        createButton(
            screen,
            gpsDisplay,
            420,
            182,
            355,
            52,
            gpsPositionEvent,
            nullptr,
            CDU_WHITE,
            CDU_MEDIUM_GRAY,
            &lv_font_montserrat_16
        );

        char satelliteText[24];
        snprintf(
            satelliteText,
            sizeof(satelliteText),
            "%u SAT - LIVE",
            liveSensors.satellites
        );

        createLabel(
            screen,
            satelliteText,
            535,
            235,
            CDU_WHITE
        );
    } else {
        const char *gpsStatus =
            sensorComputerOnline() && liveSensors.gpsOnline
                ? "SEARCHING / NO FIX"
                : "NOT AVAILABLE";

        createLabel(
            screen,
            gpsStatus,
            455,
            185,
            CDU_WHITE,
            &lv_font_montserrat_30
        );
    }

    createLabel(screen, "SET IRS POS", 500, 260, CDU_WHITE);

    createButton(
        screen,
        irsPositionSet ? irsPosition : "----",
        420,
        287,
        355,
        46,
        setIrsPositionEvent,
        nullptr,
        CDU_WHITE,
        irsPositionSet ? CDU_WHITE : CDU_MEDIUM_GRAY,
        irsPositionSet
            ? &lv_font_montserrat_16
            : &lv_font_montserrat_30
    );

    createPageButton(
        screen,
        "<INDEX",
        20,
        365,
        145,
        45,
        PageId::INDEX
    );

    createPageButton(
        screen,
        "ROUTE>",
        640,
        365,
        140,
        45,
        PageId::ROUTE
    );

    createScratchpad(screen);
}


// =====================================================
// MENU PAGE
// =====================================================


void createIndexPage()
{
    prepareScreen(
        "INIT/REF INDEX",
        "1/1"
    );

    lv_obj_t *screen =
        lv_scr_act();

    createPageButton(
        screen,
        "<IDENT",
        25,
        90,
        215,
        52,
        PageId::IDENT
    );

    createPageButton(
        screen,
        "POS INIT>",
        560,
        90,
        215,
        52,
        PageId::POS_INIT
    );

    createPageButton(
        screen,
        "<ROUTE",
        25,
        160,
        215,
        52,
        PageId::ROUTE
    );

    createPageButton(
        screen,
        "LEGS>",
        560,
        160,
        215,
        52,
        PageId::LEGS
    );

    createPageButton(
        screen,
        "<DEP/ARR",
        25,
        230,
        215,
        52,
        PageId::DEP_ARR
    );

    createPageButton(
        screen,
        "NAV DATA>",
        560,
        230,
        215,
        52,
        PageId::NAV_DATA
    );

    createPageButton(
        screen,
        "<CDU MENU",
        20,
        365,
        160,
        45,
        PageId::CDU_MENU
    );

    createPageButton(
        screen,
        "STATUS>",
        620,
        365,
        160,
        45,
        PageId::STATUS
    );

    createPageButton(
        screen,
        "<DIRECT TO",
        70,
        300,
        180,
        45,
        PageId::DIRECT_TO
    );

    createPageButton(
        screen,
        "LAT/LON NAV",
        310,
        300,
        180,
        45,
        PageId::LAT_LON
    );

    createPageButton(
        screen,
        "PROGRESS>",
        550,
        300,
        180,
        45,
        PageId::PROGRESS
    );

    createScratchpad(screen);
}

// =====================================================
// SD DEP/ARR PROCEDURES
// =====================================================

void ensureDepartureOptionsLoaded()
{
    if (strcmp(loadedDepartureAirport, routeOrigin) == 0) {
        return;
    }

    departurePageIndex = 0;
    sidTransitionPageIndex = 0;

    departureRunwayCount = 0;
    departureSidCount = 0;
    departureTransitionCount = 0;

    strncpy(
        loadedDepartureAirport,
        routeOrigin,
        sizeof(loadedDepartureAirport) - 1
    );

    loadedDepartureAirport[
        sizeof(loadedDepartureAirport) - 1
    ] = '\0';

    loadAirportRunways(
        routeOrigin,
        departureRunways,
        MAX_RUNWAY_OPTIONS,
        departureRunwayCount
    );

    loadProcedureOptions(
        routeOrigin,
        "SID",
        departureSids,
        MAX_PROCEDURE_OPTIONS,
        departureSidCount
    );
}



void ensureArrivalOptionsLoaded()
{
    if (strcmp(loadedArrivalAirport, routeDestination) == 0) {
        return;
    }

    arrivalPageIndex = 0;
    starTransitionPageIndex = 0;
    arrivalRunwayCount = 0;
    arrivalStarCount = 0;
    arrivalTransitionCount = 0;

    strncpy(
        loadedArrivalAirport,
        routeDestination,
        sizeof(loadedArrivalAirport) - 1
    );
    loadedArrivalAirport[sizeof(loadedArrivalAirport) - 1] = '\0';

    loadAirportRunways(
        routeDestination,
        arrivalRunways,
        MAX_RUNWAY_OPTIONS,
        arrivalRunwayCount
    );

    loadProcedureOptions(
        routeDestination,
        "STAR",
        arrivalStars,
        MAX_PROCEDURE_OPTIONS,
        arrivalStarCount
    );
}


void departureIndexEvent(lv_event_t *event)
{
    if (strlen(routeOrigin) == 0) {
        setScratchpad("SET ORIGIN");
        requestPage(PageId::DEP_ARR);
        return;
    }

    ensureDepartureOptionsLoaded();
    requestPage(PageId::DEPARTURES);
}

void arrivalIndexEvent(lv_event_t *event)
{
    if (strlen(routeDestination) == 0) {
        setScratchpad("SET DESTINATION");
        requestPage(PageId::DEP_ARR);
        return;
    }

    ensureArrivalOptionsLoaded();
    requestPage(PageId::ARRIVALS);
}

lv_obj_t *createSdSelectionButton(
    lv_obj_t *parent,
    const char *text,
    int x,
    int y,
    int width,
    int height,
    lv_event_cb_t callback,
    intptr_t action,
    bool selected
) {
    lv_obj_t *button = createButton(
        parent,
        text,
        x,
        y,
        width,
        height,
        callback,
        reinterpret_cast<void *>(action),
        selected ? CDU_BLACK : CDU_WHITE,
        selected ? CDU_REVERSE_VIDEO : CDU_MEDIUM_GRAY
    );

    if (selected) {
        lv_obj_set_style_bg_color(
            button,
            CDU_REVERSE_VIDEO,
            LV_PART_MAIN | LV_STATE_DEFAULT
        );
    }

    return button;
}

void dynamicProcedureSelectionEvent(lv_event_t *event)
{
    intptr_t action = reinterpret_cast<intptr_t>(
        lv_event_get_user_data(event)
    );

    PageId returnPage = PageId::DEPARTURES;

    if (action >= 100 && action < 200) {
        int index = static_cast<int>(action - 100);
        if (index < departureRunwayCount) {
            strncpy(selectedDepartureRunway, departureRunways[index], 7);
            selectedDepartureRunway[7] = '\0';
        }
    } else if (action >= 200 && action < 300) {
        int index = static_cast<int>(action - 200);
        if (index < departureSidCount) {
            strncpy(selectedSid, departureSids[index], 8);
            selectedSid[8] = '\0';

            selectedSidTransition[0] = '\0';
            sidTransitionPageIndex = 0;

            loadProcedureTransitions(
                routeOrigin,
                "SID",
                selectedSid,
                departureTransitions,
                MAX_TRANSITION_OPTIONS,
                departureTransitionCount
            );

            if (departureTransitionCount > 0) {
                returnPage = PageId::SID_TRANSITIONS;
            }

        }
    } else if (action >= 300 && action < 400) {
        int index = static_cast<int>(action - 300);
        if (index < arrivalRunwayCount) {
            strncpy(selectedArrivalRunway, arrivalRunways[index], 7);
            selectedArrivalRunway[7] = '\0';
        }
        returnPage = PageId::ARRIVALS;
    } else if (action >= 400 && action < 500) {
        int index = static_cast<int>(action - 400);
        if (index < arrivalStarCount) {
            strncpy(selectedStar, arrivalStars[index], 8);
            selectedStar[8] = '\0';

            selectedStarTransition[0] = '\0';
            starTransitionPageIndex = 0;

            loadProcedureTransitions(
                routeDestination,
                "STAR",
                selectedStar,
                arrivalTransitions,
                MAX_TRANSITION_OPTIONS,
                arrivalTransitionCount
            );

            if (arrivalTransitionCount > 0) {
                returnPage = PageId::STAR_TRANSITIONS;
            }
        }
        returnPage = PageId::ARRIVALS;
    } else if (action == 500) {
        clearDepartureSelection();
    } else if (action == 600) {
        clearArrivalSelection();
        returnPage = PageId::ARRIVALS;
    } else if (action >= 700 && action < 800) {
        int index = static_cast<int>(action - 700);
        if (index < departureTransitionCount) {
            strncpy(
                selectedSidTransition,
                departureTransitions[index],
                sizeof(selectedSidTransition) - 1
            );
            selectedSidTransition[
                sizeof(selectedSidTransition) - 1
            ] = '\0';
        }
        returnPage = PageId::SID_TRANSITIONS;
    } else if (action >= 800 && action < 900) {
        int index = static_cast<int>(action - 800);
        if (index < arrivalTransitionCount) {
            strncpy(
                selectedStarTransition,
                arrivalTransitions[index],
                sizeof(selectedStarTransition) - 1
            );
            selectedStarTransition[
                sizeof(selectedStarTransition) - 1
            ] = '\0';
        }
        returnPage = PageId::STAR_TRANSITIONS;
    }

    refreshSelectedProcedureLegs();
    routeModified = true;
    routeExecuted = false;
    clearScratchpad();

    requestPage(returnPage);
}


void procedurePageEvent(lv_event_t *event)
{
    intptr_t action = reinterpret_cast<intptr_t>(
        lv_event_get_user_data(event)
    );

    if (action == 1 && departurePageIndex > 0) {
        departurePageIndex--;
        requestPage(PageId::DEPARTURES);
    } else if (action == 2) {
        int pages = max(
            (departureRunwayCount + PROC_ROWS_PER_PAGE - 1) / PROC_ROWS_PER_PAGE,
            (departureSidCount + PROC_ROWS_PER_PAGE - 1) / PROC_ROWS_PER_PAGE
        );

        if (departurePageIndex + 1 < pages) {
            departurePageIndex++;
            requestPage(PageId::DEPARTURES);
        }
    } else if (action == 3 && arrivalPageIndex > 0) {
        arrivalPageIndex--;
        requestPage(PageId::ARRIVALS);
    } else if (action == 4) {
        int pages = max(
            (arrivalRunwayCount + PROC_ROWS_PER_PAGE - 1) / PROC_ROWS_PER_PAGE,
            (arrivalStarCount + PROC_ROWS_PER_PAGE - 1) / PROC_ROWS_PER_PAGE
        );

        if (arrivalPageIndex + 1 < pages) {
            arrivalPageIndex++;
            requestPage(PageId::ARRIVALS);
        }
    }
}


void transitionPageEvent(lv_event_t *event)
{
    intptr_t action = reinterpret_cast<intptr_t>(
        lv_event_get_user_data(event)
    );

    if (action == 1 && sidTransitionPageIndex > 0) {
        sidTransitionPageIndex--;
        requestPage(PageId::SID_TRANSITIONS);
    } else if (action == 2) {
        int pages = max(
            1,
            (departureTransitionCount + TRANS_ROWS_PER_PAGE - 1) /
                TRANS_ROWS_PER_PAGE
        );

        if (sidTransitionPageIndex + 1 < pages) {
            sidTransitionPageIndex++;
            requestPage(PageId::SID_TRANSITIONS);
        }
    } else if (action == 3 && starTransitionPageIndex > 0) {
        starTransitionPageIndex--;
        requestPage(PageId::STAR_TRANSITIONS);
    } else if (action == 4) {
        int pages = max(
            1,
            (arrivalTransitionCount + TRANS_ROWS_PER_PAGE - 1) /
                TRANS_ROWS_PER_PAGE
        );

        if (starTransitionPageIndex + 1 < pages) {
            starTransitionPageIndex++;
            requestPage(PageId::STAR_TRANSITIONS);
        }
    }
}

void createSidTransitionPage()
{
    int totalPages = max(
        1,
        (departureTransitionCount + TRANS_ROWS_PER_PAGE - 1) /
            TRANS_ROWS_PER_PAGE
    );

    if (sidTransitionPageIndex >= totalPages) {
        sidTransitionPageIndex = totalPages - 1;
    }

    char title[40];
    char counter[12];

    snprintf(
        title,
        sizeof(title),
        "%s SID TRANS",
        routeOrigin
    );

    snprintf(
        counter,
        sizeof(counter),
        "%d/%d",
        sidTransitionPageIndex + 1,
        totalPages
    );

    prepareScreen(title, counter);
    lv_obj_t *screen = lv_scr_act();

    char selectedText[80];
    snprintf(
        selectedText,
        sizeof(selectedText),
        "SID %s   RWY %s",
        strlen(selectedSid) > 0 ? selectedSid : "----",
        strlen(selectedDepartureRunway) > 0
            ? selectedDepartureRunway + 2
            : "---"
    );
    createLabel(screen, selectedText, 35, 68, CDU_WHITE);

    if (departureTransitionCount == 0) {
        createLabel(
            screen,
            "NO TRANSITIONS",
            285,
            150,
            CDU_WHITE,
            &lv_font_montserrat_30
        );
    } else {
        createLabel(screen, "TRANSITIONS", 35, 100, CDU_WHITE);

        int first = sidTransitionPageIndex * TRANS_ROWS_PER_PAGE;

        for (int row = 0; row < TRANS_ROWS_PER_PAGE; row++) {
            int optionIndex = first + row;

            if (optionIndex >= departureTransitionCount) {
                break;
            }

            char label[20];
            snprintf(
                label,
                sizeof(label),
                "<%s",
                departureTransitions[optionIndex]
            );

            createSdSelectionButton(
                screen,
                label,
                25,
                125 + row * 48,
                260,
                40,
                dynamicProcedureSelectionEvent,
                700 + optionIndex,
                strcmp(
                    selectedSidTransition,
                    departureTransitions[optionIndex]
                ) == 0
            );
        }
    }

    createButton(
        screen,
        "<PREV",
        320,
        330,
        105,
        34,
        transitionPageEvent,
        reinterpret_cast<void *>(1),
        CDU_WHITE,
        CDU_MEDIUM_GRAY
    );

    createButton(
        screen,
        "NEXT>",
        440,
        330,
        105,
        34,
        transitionPageEvent,
        reinterpret_cast<void *>(2),
        CDU_WHITE,
        CDU_MEDIUM_GRAY
    );

    createPageButton(
        screen,
        "<DEPARTURES",
        20,
        365,
        170,
        45,
        PageId::DEPARTURES
    );
    createExecButton(screen, 535, 365, PageId::SID_TRANSITIONS);
    createPageButton(screen, "ROUTE>", 650, 365, 130, 45, PageId::ROUTE);
    createScratchpad(screen);
}

void createStarTransitionPage()
{
    int totalPages = max(
        1,
        (arrivalTransitionCount + TRANS_ROWS_PER_PAGE - 1) /
            TRANS_ROWS_PER_PAGE
    );

    if (starTransitionPageIndex >= totalPages) {
        starTransitionPageIndex = totalPages - 1;
    }

    char title[40];
    char counter[12];

    snprintf(
        title,
        sizeof(title),
        "%s STAR TRANS",
        routeDestination
    );

    snprintf(
        counter,
        sizeof(counter),
        "%d/%d",
        starTransitionPageIndex + 1,
        totalPages
    );

    prepareScreen(title, counter);
    lv_obj_t *screen = lv_scr_act();

    char selectedText[80];
    snprintf(
        selectedText,
        sizeof(selectedText),
        "STAR %s   RWY %s",
        strlen(selectedStar) > 0 ? selectedStar : "----",
        strlen(selectedArrivalRunway) > 0
            ? selectedArrivalRunway + 2
            : "---"
    );
    createLabel(screen, selectedText, 35, 68, CDU_WHITE);

    if (arrivalTransitionCount == 0) {
        createLabel(
            screen,
            "NO TRANSITIONS",
            285,
            150,
            CDU_WHITE,
            &lv_font_montserrat_30
        );
    } else {
        createLabel(screen, "TRANSITIONS", 35, 100, CDU_WHITE);

        int first = starTransitionPageIndex * TRANS_ROWS_PER_PAGE;

        for (int row = 0; row < TRANS_ROWS_PER_PAGE; row++) {
            int optionIndex = first + row;

            if (optionIndex >= arrivalTransitionCount) {
                break;
            }

            char label[20];
            snprintf(
                label,
                sizeof(label),
                "<%s",
                arrivalTransitions[optionIndex]
            );

            createSdSelectionButton(
                screen,
                label,
                25,
                125 + row * 48,
                260,
                40,
                dynamicProcedureSelectionEvent,
                800 + optionIndex,
                strcmp(
                    selectedStarTransition,
                    arrivalTransitions[optionIndex]
                ) == 0
            );
        }
    }

    createButton(
        screen,
        "<PREV",
        320,
        330,
        105,
        34,
        transitionPageEvent,
        reinterpret_cast<void *>(3),
        CDU_WHITE,
        CDU_MEDIUM_GRAY
    );

    createButton(
        screen,
        "NEXT>",
        440,
        330,
        105,
        34,
        transitionPageEvent,
        reinterpret_cast<void *>(4),
        CDU_WHITE,
        CDU_MEDIUM_GRAY
    );

    createPageButton(
        screen,
        "<ARRIVALS",
        20,
        365,
        170,
        45,
        PageId::ARRIVALS
    );
    createExecButton(screen, 535, 365, PageId::STAR_TRANSITIONS);
    createPageButton(screen, "ROUTE>", 650, 365, 130, 45, PageId::ROUTE);
    createScratchpad(screen);
}

void createDepArrIndexPage()
{
    prepareScreen("DEP/ARR INDEX", "1/1");
    lv_obj_t *screen = lv_scr_act();

    char departureText[64];
    char arrivalText[64];

    snprintf(
        departureText,
        sizeof(departureText),
        "<%s DEPARTURES",
        strlen(routeOrigin) > 0 ? routeOrigin : "----"
    );

    snprintf(
        arrivalText,
        sizeof(arrivalText),
        "%s ARRIVALS>",
        strlen(routeDestination) > 0 ? routeDestination : "----"
    );

    createButton(
        screen,
        departureText,
        25,
        115,
        300,
        55,
        departureIndexEvent,
        nullptr
    );

    createButton(
        screen,
        arrivalText,
        475,
        115,
        300,
        55,
        arrivalIndexEvent,
        nullptr
    );

    createLabel(
        screen,
        "RUNWAYS / SIDS / STARS FROM SD AIRAC\n"
        "WORLD AIRPORTS WITHOUT FAA PROCEDURES SHOW NONE",
        225,
        220,
        CDU_WHITE
    );

    createPageButton(screen, "<INDEX", 20, 365, 145, 45, PageId::INDEX);
    createPageButton(screen, "ROUTE>", 640, 365, 140, 45, PageId::ROUTE);
    createScratchpad(screen);
}

void createDeparturePage()
{
    ensureDepartureOptionsLoaded();

    int runwayPages =
        (departureRunwayCount + PROC_ROWS_PER_PAGE - 1) /
        PROC_ROWS_PER_PAGE;
    int sidPages =
        (departureSidCount + PROC_ROWS_PER_PAGE - 1) /
        PROC_ROWS_PER_PAGE;
    int totalPages = max(1, max(runwayPages, sidPages));

    if (departurePageIndex >= totalPages) {
        departurePageIndex = totalPages - 1;
    }

    char title[40];
    char counter[12];

    snprintf(
        title,
        sizeof(title),
        "%s%s DEPARTURES",
        routeModified ? "MOD " : "",
        routeOrigin
    );
    snprintf(counter, sizeof(counter), "%d/%d", departurePageIndex + 1, totalPages);

    prepareScreen(title, counter);
    lv_obj_t *screen = lv_scr_act();

    createLabel(screen, "RUNWAYS", 35, 65, CDU_WHITE);
    createLabel(screen, "SIDS", 590, 65, CDU_WHITE);

    int first = departurePageIndex * PROC_ROWS_PER_PAGE;

    for (int row = 0; row < PROC_ROWS_PER_PAGE; row++) {
        int y = 88 + row * 57;
        int runwayIndex = first + row;
        int sidIndex = first + row;

        if (runwayIndex < departureRunwayCount) {
            char label[16];
            const char *runway = departureRunways[runwayIndex];
            snprintf(label, sizeof(label), "<%s", strncmp(runway, "RW", 2) == 0 ? runway + 2 : runway);

            createSdSelectionButton(
                screen,
                label,
                25,
                y,
                220,
                46,
                dynamicProcedureSelectionEvent,
                100 + runwayIndex,
                strcmp(selectedDepartureRunway, runway) == 0
            );
        }

        if (sidIndex < departureSidCount) {
            char label[16];
            snprintf(label, sizeof(label), "%s>", departureSids[sidIndex]);

            createSdSelectionButton(
                screen,
                label,
                555,
                y,
                220,
                46,
                dynamicProcedureSelectionEvent,
                200 + sidIndex,
                strcmp(selectedSid, departureSids[sidIndex]) == 0
            );
        }
    }

    if (departureRunwayCount == 0) {
        createLabel(screen, "NO RUNWAYS", 45, 110, CDU_WHITE);
    }

    if (departureSidCount == 0) {
        createLabel(screen, "NO SIDS", 630, 110, CDU_WHITE, &lv_font_montserrat_30);
    }

    char selectionInfo[120];
    snprintf(
        selectionInfo,
        sizeof(selectionInfo),
        "RWY %s   SID %s\nTRANS %s   LEGS %d",
        strlen(selectedDepartureRunway) > 0 ? selectedDepartureRunway + 2 : "---",
        strlen(selectedSid) > 0 ? selectedSid : "----",
        strlen(selectedSidTransition) > 0 ? selectedSidTransition : "----",
        selectedDepartureLegCount
    );
    createLabel(screen, selectionInfo, 270, 290, CDU_WHITE);

    createSdSelectionButton(
        screen,
        "<ERASE",
        20,
        330,
        130,
        34,
        dynamicProcedureSelectionEvent,
        500,
        false
    );

    createButton(screen, "<PREV", 165, 330, 105, 34, procedurePageEvent,
        reinterpret_cast<void *>(1), CDU_WHITE, CDU_MEDIUM_GRAY);

    if (strlen(selectedSid) > 0) {
        createPageButton(
            screen,
            "TRANS>",
            320,
            330,
            180,
            34,
            PageId::SID_TRANSITIONS
        );
    }

    createButton(screen, "NEXT>", 530, 330, 105, 34, procedurePageEvent,
        reinterpret_cast<void *>(2), CDU_WHITE, CDU_MEDIUM_GRAY);

    createPageButton(screen, "<DEP/ARR", 20, 365, 155, 45, PageId::DEP_ARR);
    createExecButton(screen, 535, 365, PageId::DEPARTURES);
    createPageButton(screen, "ROUTE>", 650, 365, 130, 45, PageId::ROUTE);
    createScratchpad(screen);
}

void createArrivalPage()
{
    ensureArrivalOptionsLoaded();

    int runwayPages =
        (arrivalRunwayCount + PROC_ROWS_PER_PAGE - 1) /
        PROC_ROWS_PER_PAGE;
    int starPages =
        (arrivalStarCount + PROC_ROWS_PER_PAGE - 1) /
        PROC_ROWS_PER_PAGE;
    int totalPages = max(1, max(runwayPages, starPages));

    if (arrivalPageIndex >= totalPages) {
        arrivalPageIndex = totalPages - 1;
    }

    char title[40];
    char counter[12];

    snprintf(
        title,
        sizeof(title),
        "%s%s ARRIVALS",
        routeModified ? "MOD " : "",
        routeDestination
    );
    snprintf(counter, sizeof(counter), "%d/%d", arrivalPageIndex + 1, totalPages);

    prepareScreen(title, counter);
    lv_obj_t *screen = lv_scr_act();

    createLabel(screen, "RUNWAYS", 35, 65, CDU_WHITE);
    createLabel(screen, "STARS", 590, 65, CDU_WHITE);

    int first = arrivalPageIndex * PROC_ROWS_PER_PAGE;

    for (int row = 0; row < PROC_ROWS_PER_PAGE; row++) {
        int y = 88 + row * 57;
        int runwayIndex = first + row;
        int starIndex = first + row;

        if (runwayIndex < arrivalRunwayCount) {
            char label[16];
            const char *runway = arrivalRunways[runwayIndex];
            snprintf(label, sizeof(label), "<%s", strncmp(runway, "RW", 2) == 0 ? runway + 2 : runway);

            createSdSelectionButton(
                screen,
                label,
                25,
                y,
                220,
                46,
                dynamicProcedureSelectionEvent,
                300 + runwayIndex,
                strcmp(selectedArrivalRunway, runway) == 0
            );
        }

        if (starIndex < arrivalStarCount) {
            char label[16];
            snprintf(label, sizeof(label), "%s>", arrivalStars[starIndex]);

            createSdSelectionButton(
                screen,
                label,
                555,
                y,
                220,
                46,
                dynamicProcedureSelectionEvent,
                400 + starIndex,
                strcmp(selectedStar, arrivalStars[starIndex]) == 0
            );
        }
    }

    if (arrivalRunwayCount == 0) {
        createLabel(screen, "NO RUNWAYS", 45, 110, CDU_WHITE);
    }

    if (arrivalStarCount == 0) {
        createLabel(screen, "NO STARS", 625, 110, CDU_WHITE, &lv_font_montserrat_30);
    }

    char selectionInfo[120];
    snprintf(
        selectionInfo,
        sizeof(selectionInfo),
        "RWY %s   STAR %s\nTRANS %s   LEGS %d",
        strlen(selectedArrivalRunway) > 0 ? selectedArrivalRunway + 2 : "---",
        strlen(selectedStar) > 0 ? selectedStar : "----",
        strlen(selectedStarTransition) > 0 ? selectedStarTransition : "----",
        selectedArrivalLegCount
    );
    createLabel(screen, selectionInfo, 270, 290, CDU_WHITE);

    createSdSelectionButton(
        screen,
        "<ERASE",
        20,
        330,
        130,
        34,
        dynamicProcedureSelectionEvent,
        600,
        false
    );

    createButton(screen, "<PREV", 165, 330, 105, 34, procedurePageEvent,
        reinterpret_cast<void *>(3), CDU_WHITE, CDU_MEDIUM_GRAY);

    if (strlen(selectedStar) > 0) {
        createPageButton(
            screen,
            "TRANS>",
            320,
            330,
            180,
            34,
            PageId::STAR_TRANSITIONS
        );
    }

    createButton(screen, "NEXT>", 530, 330, 105, 34, procedurePageEvent,
        reinterpret_cast<void *>(4), CDU_WHITE, CDU_MEDIUM_GRAY);

    createPageButton(screen, "<DEP/ARR", 20, 365, 155, 45, PageId::DEP_ARR);
    createExecButton(screen, 535, 365, PageId::ARRIVALS);
    createPageButton(screen, "ROUTE>", 650, 365, 130, 45, PageId::ROUTE);
    createScratchpad(screen);
}

// =====================================================
// MANUAL LAT/LON NAVIGATION PAGE
// =====================================================

void createLatLonPage()
{
    prepareScreen("LAT/LON NAV", "1/1");
    lv_obj_t *screen = lv_scr_act();

    GeoPosition effectiveFrom = manualFromPosition;
    bool usingLiveGps = false;

    if (liveGpsPositionAvailable()) {
        effectiveFrom.valid = true;
        effectiveFrom.latitude = liveSensors.latitude;
        effectiveFrom.longitude = liveSensors.longitude;

        formatBoeingLatLon(
            effectiveFrom.latitude,
            effectiveFrom.longitude,
            effectiveFrom.display,
            sizeof(effectiveFrom.display)
        );

        formatLiveGpsCompact(
            effectiveFrom.latitude,
            effectiveFrom.longitude,
            effectiveFrom.entry,
            sizeof(effectiveFrom.entry),
            false
        );

        usingLiveGps = true;
    }

    createLabel(
        screen,
        usingLiveGps ? "FROM POSITION / GPS" : "FROM POSITION",
        35,
        70,
        CDU_WHITE
    );

    if (usingLiveGps) {
        createButton(
            screen,
            effectiveFrom.display,
            25,
            95,
            345,
            52,
            gpsPositionEvent,
            nullptr,
            CDU_WHITE,
            CDU_MEDIUM_GRAY,
            &lv_font_montserrat_16
        );
    } else {
        createInputField(
            screen,
            manualFromPosition.valid
                ? manualFromPosition.display
                : "ENTER FROM",
            25,
            95,
            345,
            52,
            InputTarget::LATLON_FROM
        );
    }

    createLabel(screen, "TO POSITION", 430, 70, CDU_WHITE);
    createInputField(
        screen,
        manualToPosition.valid
            ? manualToPosition.display
            : "ENTER TO",
        420,
        95,
        355,
        52,
        InputTarget::LATLON_TO
    );

    createLabel(
        screen,
        "ENTRY FORMAT\nN3952.7W07513.8",
        35,
        185,
        CDU_WHITE
    );

    if (effectiveFrom.valid && manualToPosition.valid) {
        const double distance = greatCircleDistanceNm(
            effectiveFrom.latitude,
            effectiveFrom.longitude,
            manualToPosition.latitude,
            manualToPosition.longitude
        );

        const double course = initialTrueCourse(
            effectiveFrom.latitude,
            effectiveFrom.longitude,
            manualToPosition.latitude,
            manualToPosition.longitude
        );

        char navResult[190];

        snprintf(
            navResult,
            sizeof(navResult),
            "DTK TRUE     %03.0f DEG\n"
            "DISTANCE     %.2f NM\n"
            "             %.2f MI",
            course,
            distance,
            distance * 1.150779
        );

        createLabel(
            screen,
            navResult,
            430,
            185,
            CDU_WHITE,
            &lv_font_montserrat_30
        );
    } else {
        createLabel(
            screen,
            "ENTER DESTINATION\nFOR DTK AND DISTANCE",
            430,
            205,
            CDU_WHITE
        );
    }

    char sourceStatus[96];

    if (usingLiveGps) {
        snprintf(
            sourceStatus,
            sizeof(sourceStatus),
            "GPS FROM ACTIVE - %u SAT - GS %.1f KT - TRK %03.0f",
            liveSensors.satellites,
            liveSensors.groundSpeedKt,
            liveSensors.trackDeg
        );
    } else {
        snprintf(
            sourceStatus,
            sizeof(sourceStatus),
            "GPS FROM UNAVAILABLE - MANUAL FROM ACTIVE"
        );
    }

    createLabel(
        screen,
        sourceStatus,
        120,
        310,
        CDU_WHITE
    );

    createPageButton(screen, "<INDEX", 20, 365, 140, 45, PageId::INDEX);
    createPageButton(screen, "ROUTE>", 640, 365, 140, 45, PageId::ROUTE);
    createScratchpad(screen);
}


// =====================================================
// DIRECT TO / PROGRESS NAVIGATION
// =====================================================

bool getFmcPresentPosition(
    double &latitude,
    double &longitude,
    bool &usingGps
) {
    if (liveGpsPositionAvailable()) {
        latitude = liveSensors.latitude;
        longitude = liveSensors.longitude;
        usingGps = true;
        return true;
    }

    if (irsPositionSet) {
        GeoPosition initializedPosition = {};

        if (parseBoeingLatLon(irsPosition, initializedPosition)) {
            latitude = initializedPosition.latitude;
            longitude = initializedPosition.longitude;
            usingGps = false;
            return true;
        }
    }

    usingGps = false;
    return false;
}

bool getProgressTarget(
    const char *&ident,
    double &latitude,
    double &longitude,
    const char *&targetSource
) {
    if (directToTarget.valid) {
        ident = directToTarget.ident;
        latitude = directToTarget.latitude;
        longitude = directToTarget.longitude;
        targetSource = "DIRECT";
        return true;
    }

    if (
        strlen(routeDestination) > 0 &&
        (
            navDataReady ||
            routeDestinationLatitudeE7 != 0 ||
            routeDestinationLongitudeE7 != 0
        )
    ) {
        ident = routeDestination;
        latitude =
            static_cast<double>(routeDestinationLatitudeE7) / 10000000.0;
        longitude =
            static_cast<double>(routeDestinationLongitudeE7) / 10000000.0;
        targetSource = "RTE DEST";
        return true;
    }

    return false;
}

void formatEte(double distanceNm, float groundSpeedKt, char *output, size_t size)
{
    if (groundSpeedKt < 0.5F) {
        snprintf(output, size, "--:--");
        return;
    }

    uint32_t totalSeconds = static_cast<uint32_t>(
        (distanceNm / static_cast<double>(groundSpeedKt)) * 3600.0 + 0.5
    );

    uint32_t hours = totalSeconds / 3600U;
    uint32_t minutes = (totalSeconds % 3600U + 30U) / 60U;

    if (minutes >= 60U) {
        hours++;
        minutes = 0U;
    }

    if (hours > 99U) {
        snprintf(output, size, ">99H");
        return;
    }

    snprintf(output, size, "%02lu:%02lu",
        static_cast<unsigned long>(hours),
        static_cast<unsigned long>(minutes));
}

void formatEtaZulu(
    double distanceNm,
    float groundSpeedKt,
    char *output,
    size_t size
) {
    if (
        !liveGpsPositionAvailable() ||
        !liveSensors.utcValid ||
        groundSpeedKt < 0.5F
    ) {
        snprintf(output, size, "----Z");
        return;
    }

    uint32_t currentSeconds =
        static_cast<uint32_t>(liveSensors.utcHour) * 3600U +
        static_cast<uint32_t>(liveSensors.utcMinute) * 60U +
        static_cast<uint32_t>(liveSensors.utcSecond);

    uint32_t enrouteSeconds = static_cast<uint32_t>(
        (distanceNm / static_cast<double>(groundSpeedKt)) * 3600.0 + 0.5
    );

    uint32_t etaSeconds = (currentSeconds + enrouteSeconds) % 86400U;
    uint32_t etaHour = etaSeconds / 3600U;
    uint32_t etaMinute = (etaSeconds % 3600U) / 60U;

    snprintf(
        output,
        size,
        "%02lu%02luZ",
        static_cast<unsigned long>(etaHour),
        static_cast<unsigned long>(etaMinute)
    );
}

void clearDirectToEvent(lv_event_t *event)
{
    (void)event;
    directToTarget = {};
    clearScratchpad();
    requestPage(PageId::DIRECT_TO);
}

void createDirectToPage()
{
    prepareScreen("DIRECT TO", "1/1");
    lv_obj_t *screen = lv_scr_act();

    createLabel(screen, "WAYPOINT / COORDINATE", 35, 70, CDU_WHITE);

    createInputField(
        screen,
        directToTarget.valid
            ? directToTarget.ident
            : "ENTER WPT / LAT-LON",
        25,
        95,
        315,
        52,
        InputTarget::DIRECT_TO
    );

    if (directToTarget.valid) {
        createLabel(
            screen,
            directToTarget.display,
            35,
            155,
            CDU_WHITE,
            &lv_font_montserrat_16
        );
    } else {
        createLabel(
            screen,
            "EXAMPLE: MXE\nOR N3952.7W07513.8",
            35,
            155,
            CDU_WHITE
        );
    }

    double presentLatitude = 0.0;
    double presentLongitude = 0.0;
    bool usingGps = false;
    bool positionValid = getFmcPresentPosition(
        presentLatitude,
        presentLongitude,
        usingGps
    );

    createLabel(screen, "PRESENT POS", 440, 70, CDU_WHITE);
    createLabel(
        screen,
        positionValid
            ? (usingGps ? "GPS" : "POS INIT")
            : "NOT SET",
        440,
        97,
        CDU_WHITE,
        &lv_font_montserrat_30
    );

    if (positionValid && directToTarget.valid) {
        double distance = greatCircleDistanceNm(
            presentLatitude,
            presentLongitude,
            directToTarget.latitude,
            directToTarget.longitude
        );

        double desiredTrack = initialTrueCourse(
            presentLatitude,
            presentLongitude,
            directToTarget.latitude,
            directToTarget.longitude
        );

        char navText[160];
        snprintf(
            navText,
            sizeof(navText),
            "DTK TRUE     %03.0f DEG\n"
            "DTG          %.1f NM",
            desiredTrack,
            distance
        );

        createLabel(
            screen,
            navText,
            440,
            165,
            CDU_WHITE,
            &lv_font_montserrat_30
        );
    } else if (!positionValid) {
        createLabel(
            screen,
            "SET POS INIT\nFOR DTK / DTG",
            440,
            175,
            CDU_WHITE,
            &lv_font_montserrat_30
        );
    } else {
        createLabel(
            screen,
            "ENTER DIRECT-TO\nTARGET",
            440,
            175,
            CDU_WHITE,
            &lv_font_montserrat_30
        );
    }

    if (directToTarget.valid) {
        createButton(
            screen,
            "<CLEAR DIRECT",
            25,
            285,
            220,
            45,
            clearDirectToEvent,
            nullptr
        );
    }

    createPageButton(screen, "<INDEX", 20, 365, 140, 45, PageId::INDEX);
    createPageButton(screen, "PROGRESS>", 610, 365, 170, 45, PageId::PROGRESS);
    createScratchpad(screen);
}

void createProgressPage()
{
    prepareScreen("PROGRESS", "1/1");
    lv_obj_t *screen = lv_scr_act();

    const char *targetIdent = nullptr;
    const char *targetSource = nullptr;
    double targetLatitude = 0.0;
    double targetLongitude = 0.0;

    bool targetValid = getProgressTarget(
        targetIdent,
        targetLatitude,
        targetLongitude,
        targetSource
    );

    double presentLatitude = 0.0;
    double presentLongitude = 0.0;
    bool usingGps = false;
    bool positionValid = getFmcPresentPosition(
        presentLatitude,
        presentLongitude,
        usingGps
    );

    createLabel(screen, "TO", 35, 65, CDU_WHITE);
    createLabel(
        screen,
        targetValid ? targetIdent : "----",
        35,
        90,
        CDU_WHITE,
        &lv_font_montserrat_30
    );

    createLabel(screen, "TARGET", 210, 65, CDU_WHITE);
    createLabel(
        screen,
        targetValid ? targetSource : "----",
        210,
        90,
        CDU_WHITE,
        &lv_font_montserrat_30
    );

    createLabel(screen, "POS SOURCE", 555, 65, CDU_WHITE);
    createLabel(
        screen,
        positionValid
            ? (usingGps ? "GPS" : "POS INIT")
            : "NOT SET",
        555,
        90,
        CDU_WHITE,
        &lv_font_montserrat_30
    );

    char dtgText[24] = "---.- NM";
    char dtkText[24] = "--- DEG";
    char gsText[24] = "--- KT";
    char actualTrackText[24] = "--- DEG";
    char eteText[24] = "--:--";
    char etaText[24] = "----Z";

    if (positionValid && targetValid) {
        double distance = greatCircleDistanceNm(
            presentLatitude,
            presentLongitude,
            targetLatitude,
            targetLongitude
        );

        double desiredTrack = initialTrueCourse(
            presentLatitude,
            presentLongitude,
            targetLatitude,
            targetLongitude
        );

        snprintf(dtgText, sizeof(dtgText), "%.1f NM", distance);
        snprintf(dtkText, sizeof(dtkText), "%03.0f DEG", desiredTrack);

        if (liveGpsPositionAvailable()) {
            snprintf(
                gsText,
                sizeof(gsText),
                "%.1f KT",
                liveSensors.groundSpeedKt
            );

            snprintf(
                actualTrackText,
                sizeof(actualTrackText),
                "%03.0f DEG",
                liveSensors.trackDeg
            );

            formatEte(
                distance,
                liveSensors.groundSpeedKt,
                eteText,
                sizeof(eteText)
            );

            formatEtaZulu(
                distance,
                liveSensors.groundSpeedKt,
                etaText,
                sizeof(etaText)
            );
        }
    }

    createLabel(screen, "DTG", 35, 155, CDU_WHITE);
    createLabel(screen, dtgText, 35, 182, CDU_WHITE, &lv_font_montserrat_30);

    createLabel(screen, "DTK TRUE", 295, 155, CDU_WHITE);
    createLabel(screen, dtkText, 295, 182, CDU_WHITE, &lv_font_montserrat_30);

    createLabel(screen, "GS", 590, 155, CDU_WHITE);
    createLabel(screen, gsText, 590, 182, CDU_WHITE, &lv_font_montserrat_30);

    createLabel(screen, "GPS TRK", 35, 245, CDU_WHITE);
    createLabel(
        screen,
        actualTrackText,
        35,
        272,
        CDU_WHITE,
        &lv_font_montserrat_30
    );

    createLabel(screen, "ETE", 315, 245, CDU_WHITE);
    createLabel(screen, eteText, 315, 272, CDU_WHITE, &lv_font_montserrat_30);

    createLabel(screen, "ETA", 590, 245, CDU_WHITE);
    createLabel(screen, etaText, 590, 272, CDU_WHITE, &lv_font_montserrat_30);

    if (!targetValid) {
        createLabel(
            screen,
            "NO DIRECT-TO / DESTINATION SET",
            250,
            325,
            CDU_WHITE
        );
    } else if (!positionValid) {
        createLabel(
            screen,
            "SET POS INIT OR ACQUIRE GPS",
            270,
            325,
            CDU_WHITE
        );
    } else if (!usingGps) {
        createLabel(
            screen,
            "NO GPS FIX - GS / GPS TRK / ETE / ETA UNAVAILABLE",
            145,
            325,
            CDU_WHITE
        );
    }

    createPageButton(screen, "<DIRECT TO", 20, 365, 175, 45, PageId::DIRECT_TO);
    createPageButton(screen, "INDEX>", 650, 365, 130, 45, PageId::INDEX);
    createScratchpad(screen);
}


// =====================================================
// ROUTE PAGE
// =====================================================

void execEvent(lv_event_t *event)
{
    if (!routeModified) {
        return;
    }

    routeModified = false;
    routeExecuted = true;
    clearScratchpad();

    PageId returnPage =
        static_cast<PageId>(
            reinterpret_cast<intptr_t>(
                lv_event_get_user_data(event)
            )
        );

    requestPage(returnPage);
}

void createRoutePage()
{
    prepareScreen(
        routeModified
            ? "MOD RTE 1"
            : (routeExecuted
                ? "ACT RTE 1"
                : "RTE 1"),
        "1/1"
    );

    lv_obj_t *screen =
        lv_scr_act();

    createLabel(
        screen,
        "ORIGIN",
        30,
        72,
        CDU_WHITE
    );

    createInputField(
        screen,
        routeOrigin,
        25,
        98,
        190,
        50,
        InputTarget::ORIGIN
    );

    createLabel(
        screen,
        "DESTINATION",
        585,
        72,
        CDU_WHITE
    );

    createInputField(
        screen,
        routeDestination,
        585,
        98,
        190,
        50,
        InputTarget::DESTINATION
    );

    if (routeModified) {
        // Boeing-style reverse-video MOD annunciation:
        // light gray background with black text.
        lv_obj_t *modHighlight = lv_obj_create(screen);

        lv_obj_set_size(
            modHighlight,
            92,
            42
        );

        lv_obj_align(
            modHighlight,
            LV_ALIGN_TOP_LEFT,
            354,
            96
        );

        lv_obj_set_style_bg_color(
            modHighlight,
            CDU_REVERSE_VIDEO,
            0
        );

        lv_obj_set_style_bg_opa(
            modHighlight,
            LV_OPA_COVER,
            0
        );

        lv_obj_set_style_border_width(
            modHighlight,
            0,
            0
        );

        lv_obj_set_style_radius(
            modHighlight,
            0,
            0
        );

        lv_obj_set_style_pad_all(
            modHighlight,
            0,
            0
        );

        lv_obj_clear_flag(
            modHighlight,
            LV_OBJ_FLAG_SCROLLABLE
        );

        lv_obj_t *modLabel =
            lv_label_create(modHighlight);

        lv_label_set_text(
            modLabel,
            "MOD"
        );

        lv_obj_set_style_text_color(
            modLabel,
            CDU_BLACK,
            0
        );

        lv_obj_set_style_text_font(
            modLabel,
            &lv_font_montserrat_30,
            0
        );

        lv_obj_center(modLabel);
    } else {
        createLabel(
            screen,
            routeExecuted
                ? "ACTIVE"
                : "PREVIEW",
            355,
            110,
            CDU_WHITE
        );
    }

    createLabel(
        screen,
        "ROUTE LEGS",
        30,
        170,
        CDU_WHITE
    );

    int y = 202;

    if (routeLegCount == 0) {
        createLabel(
            screen,
            "NO ROUTE LEGS",
            35,
            y,
            CDU_WHITE
        );
    } else {
        for (
            int index = 0;
            index < routeLegCount;
            index++
        ) {
            char line[32];

            snprintf(
                line,
                sizeof(line),
                "%02d   %s",
                index + 1,
                routeLegs[index]
            );

            createLabel(
                screen,
                line,
                35,
                y,
                CDU_WHITE
            );

            y += 34;
        }
    }

    createLabel(
        screen,
        "ADD WAYPOINT",
        560,
        175,
        CDU_WHITE
    );

    createInputField(
        screen,
        "TAP TO ENTER",
        550,
        202,
        225,
        50,
        InputTarget::WAYPOINT
    );

    createPageButton(
        screen,
        "<INDEX",
        20,
        365,
        130,
        45,
        PageId::INDEX
    );

    createExecButton(
        screen,
        535,
        365,
        PageId::ROUTE
    );

    createPageButton(
        screen,
        "LEGS>",
        650,
        365,
        130,
        45,
        PageId::LEGS
    );

    createScratchpad(screen);
}

// =====================================================
// LEGS PAGE
// =====================================================

lv_obj_t *createLegTouchRow(
    lv_obj_t *parent,
    const char *ident,
    const char *rightText,
    int y,
    lv_color_t textColor,
    int routeIndex
)
{
    lv_obj_t *row =
        lv_btn_create(parent);

    lv_obj_set_size(
        row,
        740,
        50
    );

    lv_obj_align(
        row,
        LV_ALIGN_TOP_LEFT,
        30,
        y
    );

    lv_obj_set_style_bg_opa(
        row,
        LV_OPA_TRANSP,
        LV_PART_MAIN |
            LV_STATE_DEFAULT
    );

    lv_obj_set_style_bg_color(
        row,
        CDU_DARK_GRAY,
        LV_PART_MAIN |
            LV_STATE_PRESSED
    );

    lv_obj_set_style_bg_opa(
        row,
        LV_OPA_COVER,
        LV_PART_MAIN |
            LV_STATE_PRESSED
    );

    lv_obj_set_style_border_width(
        row,
        0,
        LV_PART_MAIN
    );

    lv_obj_set_style_radius(
        row,
        0,
        LV_PART_MAIN
    );

    lv_obj_set_style_pad_all(
        row,
        0,
        LV_PART_MAIN
    );

    lv_obj_add_event_cb(
        row,
        legWaypointEvent,
        LV_EVENT_CLICKED,
        reinterpret_cast<void *>(
            static_cast<intptr_t>(
                routeIndex
            )
        )
    );

    lv_obj_t *identLabel =
        lv_label_create(row);

    lv_label_set_text(
        identLabel,
        ident
    );

    lv_obj_set_style_text_color(
        identLabel,
        textColor,
        0
    );

    lv_obj_set_style_text_font(
        identLabel,
        &lv_font_montserrat_30,
        0
    );

    lv_obj_align(
        identLabel,
        LV_ALIGN_LEFT_MID,
        8,
        0
    );

    lv_obj_t *rightLabel =
        lv_label_create(row);

    lv_label_set_text(
        rightLabel,
        rightText
    );

    lv_obj_set_style_text_color(
        rightLabel,
        textColor,
        0
    );

    lv_obj_set_style_text_font(
        rightLabel,
        &lv_font_montserrat_16,
        0
    );

    lv_obj_align(
        rightLabel,
        LV_ALIGN_RIGHT_MID,
        -8,
        0
    );

    return row;
}


lv_obj_t *createDiscontinuityTouchRow(
    lv_obj_t *parent,
    int y,
    int discontinuityIndex
)
{
    lv_obj_t *row =
        lv_btn_create(parent);

    lv_obj_set_size(
        row,
        740,
        50
    );

    lv_obj_align(
        row,
        LV_ALIGN_TOP_LEFT,
        30,
        y
    );

    lv_obj_set_style_bg_opa(
        row,
        LV_OPA_TRANSP,
        LV_PART_MAIN |
            LV_STATE_DEFAULT
    );

    lv_obj_set_style_bg_color(
        row,
        CDU_DARK_GRAY,
        LV_PART_MAIN |
            LV_STATE_PRESSED
    );

    lv_obj_set_style_bg_opa(
        row,
        LV_OPA_COVER,
        LV_PART_MAIN |
            LV_STATE_PRESSED
    );

    lv_obj_set_style_border_width(
        row,
        0,
        LV_PART_MAIN
    );

    lv_obj_set_style_radius(
        row,
        0,
        LV_PART_MAIN
    );

    lv_obj_set_style_pad_all(
        row,
        0,
        LV_PART_MAIN
    );

    lv_obj_add_event_cb(
        row,
        discontinuityEvent,
        LV_EVENT_CLICKED,
        reinterpret_cast<void *>(
            static_cast<intptr_t>(
                discontinuityIndex
            )
        )
    );

    lv_obj_t *label =
        lv_label_create(row);

    lv_label_set_text(
        label,
        "----- ROUTE DISCONTINUITY -----"
    );

    lv_obj_set_style_text_color(
        label,
        CDU_WHITE,
        0
    );

    lv_obj_set_style_text_font(
        label,
        &lv_font_montserrat_16,
        0
    );

    lv_obj_center(label);

    return row;
}

void legsPageEvent(lv_event_t *event)
{
    intptr_t action = reinterpret_cast<intptr_t>(
        lv_event_get_user_data(event)
    );

    int totalItems =
        routeLegCount +
        (strlen(routeDestination) > 0 ? 1 : 0) +
        (routeDiscontinuityIndex >= 0 ? 1 : 0);

    int totalPages = max(1, (totalItems + 4) / 5);

    if (action == -1 && legsPageIndex > 0) {
        legsPageIndex--;
    } else if (action == 1 && legsPageIndex + 1 < totalPages) {
        legsPageIndex++;
    }

    requestPage(PageId::LEGS);
}

void createLegsPage()
{
    int totalItems =
        routeLegCount +
        (strlen(routeDestination) > 0 ? 1 : 0) +
        (routeDiscontinuityIndex >= 0 ? 1 : 0);

    int totalPages = max(1, (totalItems + 4) / 5);

    if (legsPageIndex >= totalPages) {
        legsPageIndex = totalPages - 1;
    }

    char counter[12];
    snprintf(counter, sizeof(counter), "%d/%d", legsPageIndex + 1, totalPages);

    prepareScreen(
        routeModified
            ? "MOD RTE 1 LEGS"
            : (routeExecuted
                ? "ACT RTE 1 LEGS"
                : "RTE 1 LEGS"),
        counter
    );

    lv_obj_t *screen = lv_scr_act();

    createLabel(screen, "WAYPOINT", 35, 62, CDU_WHITE);
    createLabel(screen, "COURSE / DISTANCE", 530, 62, CDU_WHITE);

    int y = 82;
    int startItem = legsPageIndex * 5;
    int endItem = startItem + 5;
    int logicalItem = 0;
    int rowsDisplayed = 0;

    if (totalItems == 0) {
        createLabel(
            screen,
            "NO ACTIVE ROUTE",
            40,
            105,
            CDU_WHITE,
            &lv_font_montserrat_30
        );
    } else {
        for (int index = 0; index <= routeLegCount; index++) {
            if (routeDiscontinuityIndex == index) {
                if (logicalItem >= startItem && logicalItem < endItem) {
                    createDiscontinuityTouchRow(screen, y, index);
                    y += 54;
                    rowsDisplayed++;
                }
                logicalItem++;
            }

            if (index < routeLegCount) {
                if (logicalItem >= startItem && logicalItem < endItem) {
                    bool active = index == activeLegIndex;
                    lv_color_t rowColor = active ? CDU_MAGENTA : CDU_WHITE;

                    createLegTouchRow(
                        screen,
                        routeLegs[index],
                        "--- DEG    ---.- NM",
                        y,
                        rowColor,
                        index
                    );

                    y += 54;
                    rowsDisplayed++;
                }
                logicalItem++;
            }
        }

        if (strlen(routeDestination) > 0) {
            if (logicalItem >= startItem && logicalItem < endItem) {
                bool destinationActive = routeLegCount == activeLegIndex;

                createLegTouchRow(
                    screen,
                    routeDestination,
                    "--- DEG    ---.- NM",
                    y,
                    destinationActive ? CDU_MAGENTA : CDU_WHITE,
                    -1
                );
            }
        }
    }

    createButton(
        screen,
        "<PREV",
        180,
        330,
        105,
        34,
        legsPageEvent,
        reinterpret_cast<void *>(-1),
        CDU_WHITE,
        CDU_MEDIUM_GRAY
    );

    createButton(
        screen,
        "NEXT>",
        515,
        330,
        105,
        34,
        legsPageEvent,
        reinterpret_cast<void *>(1),
        CDU_WHITE,
        CDU_MEDIUM_GRAY
    );

    createPageButton(screen, "<ROUTE", 20, 365, 145, 45, PageId::ROUTE);
    createExecButton(screen, 510, 365, PageId::LEGS);
    createPageButton(screen, "NAV DATA>", 620, 365, 160, 45, PageId::NAV_DATA);
    createScratchpad(screen);
}

// =====================================================
// NAV DATA PAGE
// =====================================================

void createNavDataPage()
{
    prepareScreen(
        "NAV DATA",
        "1/1"
    );

    lv_obj_t *screen = lv_scr_act();

    char databaseInfo[220];

    if (navDataReady) {
        snprintf(
            databaseInfo,
            sizeof(databaseInfo),
            "SOURCE        FAA + WORLD / SD\n"
            "CYCLE         %s  %s\n"
            "AIRPORT KEYS  %lu\n"
            "POINT KEYS    %lu\n"
            "PROC AIRPORTS %lu",
            navCycle,
            navStatus,
            static_cast<unsigned long>(navAirportIndexCount),
            static_cast<unsigned long>(navPointIndexCount),
            static_cast<unsigned long>(navProcedureAirportCount)
        );
    } else {
        snprintf(
            databaseInfo,
            sizeof(databaseInfo),
            "SOURCE        BUILT-IN FALLBACK\n"
            "SD CARD       %s\n"
            "NAVDATA       %s\n"
            "DATABASE      TEST ONLY",
            sdMounted ? "MOUNTED" : "NOT MOUNTED",
            navStatus
        );
    }

    createLabel(
        screen,
        databaseInfo,
        35,
        72,
        CDU_WHITE
    );

    createLabel(
        screen,
        "IDENT LOOKUP",
        35,
        245,
        CDU_WHITE
    );

    createInputField(
        screen,
        "TAP TO SEARCH",
        30,
        270,
        250,
        50,
        InputTarget::NAV_LOOKUP
    );

    if (lastLookupValid) {
        char result[220];

        snprintf(
            result,
            sizeof(result),
            "IDENT  %s\n"
            "TYPE   %s\n"
            "LAT    %.6f\n"
            "LON    %.6f\n"
            "%s",
            lastLookupIdent,
            lastLookupType,
            lastLookupLatitudeE7 / 10000000.0,
            lastLookupLongitudeE7 / 10000000.0,
            strlen(lastLookupName) > 0
                ? lastLookupName
                : "VALID"
        );

        createLabel(
            screen,
            result,
            385,
            225,
            CDU_WHITE
        );
    } else {
        createLabel(
            screen,
            "NO NAV OBJECT SELECTED",
            430,
            270,
            CDU_WHITE
        );
    }

    createPageButton(
        screen,
        "<INDEX",
        20,
        365,
        140,
        45,
        PageId::INDEX
    );

    createPageButton(
        screen,
        "ROUTE>",
        640,
        365,
        140,
        45,
        PageId::ROUTE
    );

    createScratchpad(screen);
}

// =====================================================
// STATUS PAGE
// =====================================================

void createStatusPage()
{
    prepareScreen("FMC STATUS", "1/1");
    lv_obj_t *screen = lv_scr_act();

    const bool sensorOnline = sensorComputerOnline();

    const char *gpsState = "OFF";
    const char *baroState = "OFF";
    const char *ahrsState = "OFF";

    if (sensorOnline) {
        if (liveSensors.gpsOnline) {
            gpsState = liveSensors.gpsFix ? "FIX" : "NO FIX";
        }

        baroState = liveSensors.bmeOnline ? "OK" : "OFF";
        ahrsState = liveSensors.bnoOnline ? "OK" : "OFF";
    }

    char sensorSystems[64];

    snprintf(
        sensorSystems,
        sizeof(sensorSystems),
        "%s / %s / %s",
        gpsState,
        baroState,
        ahrsState
    );

    char statusText[360];

    snprintf(
        statusText,
        sizeof(statusText),
        "DISPLAY SYSTEM        OK\n"
        "TOUCH CONTROLLER      OK\n"
        "FMC CORE              ACTIVE\n"
        "SD CARD               %s\n"
        "NAV DATABASE          %s\n"
        "AIRAC                 %s %s\n"
        "SENSOR COMPUTER       %s\n"
        "GPS / BARO / AHRS     %s",
        sdMounted ? "MOUNTED" : "OFFLINE",
        navDataReady ? "ONLINE" : "FALLBACK",
        navCycle,
        navStatus,
        sensorOnline ? "ONLINE" : "OFFLINE",
        sensorSystems
    );

    createLabel(
        screen,
        statusText,
        55,
        78,
        CDU_WHITE,
        &lv_font_montserrat_30
    );

    createPageButton(
        screen,
        "<INDEX",
        20,
        365,
        140,
        45,
        PageId::INDEX
    );

    createPageButton(
        screen,
        "NAV DATA>",
        620,
        365,
        160,
        45,
        PageId::NAV_DATA
    );

    createScratchpad(screen);
}


// =====================================================
// DUPLICATE-WAYPOINT PAGE
// =====================================================

void desiredWaypointEvent(lv_event_t *event)
{
    int candidateIndex =
        static_cast<int>(
            reinterpret_cast<intptr_t>(
                lv_event_get_user_data(event)
            )
        );

    applyNavCandidate(
        candidateIndex,
        pendingMatchTarget
    );

    requestPage(
        pageForInputTarget(
            pendingMatchTarget
        )
    );

    pendingMatchTarget = InputTarget::NONE;
}

void createSelectWaypointPage()
{
    prepareScreen(
        "SELECT DESIRED WPT",
        "1/1"
    );

    lv_obj_t *screen = lv_scr_act();

    createLabel(
        screen,
        "MULTIPLE DATABASE MATCHES",
        245,
        65,
        CDU_WHITE
    );

    int y = 95;

    for (
        int index = 0;
        index < navMatchCount && index < 5;
        index++
    ) {
        const NavCandidate &record = navMatches[index];
        char textValue[100];

        snprintf(
            textValue,
            sizeof(textValue),
            "%s  %-7s  %s",
            record.ident,
            navTypeText(record.type),
            strlen(record.region) > 0
                ? record.region
                : "--"
        );

        createButton(
            screen,
            textValue,
            120,
            y,
            560,
            48,
            desiredWaypointEvent,
            reinterpret_cast<void *>(
                static_cast<intptr_t>(index)
            )
        );

        y += 54;
    }

    createPageButton(
        screen,
        "<CANCEL",
        20,
        365,
        150,
        45,
        pageForInputTarget(
            pendingMatchTarget
        )
    );

    createScratchpad(screen);
}

// =====================================================
// PAGE CONTROL
// =====================================================

void requestPage(PageId page)
{
    requestedPage = page;
    redrawRequested = true;
}

void renderPage(PageId page)
{
    switch (page) {
        case PageId::CDU_MENU:
            createCduMenuPage();
            break;

        case PageId::IDENT:
            createIdentPage();
            break;

        case PageId::POS_INIT:
            createPosInitPage();
            break;

        case PageId::INDEX:
            createIndexPage();
            break;

        case PageId::DEP_ARR:
            createDepArrIndexPage();
            break;

        case PageId::DEPARTURES:
            createDeparturePage();
            break;

        case PageId::ARRIVALS:
            createArrivalPage();
            break;

        case PageId::SID_TRANSITIONS:
            createSidTransitionPage();
            break;

        case PageId::STAR_TRANSITIONS:
            createStarTransitionPage();
            break;

        case PageId::ROUTE:
            createRoutePage();
            break;

        case PageId::LEGS:
            createLegsPage();
            break;

        case PageId::DIRECT_TO:
            createDirectToPage();
            break;

        case PageId::PROGRESS:
            createProgressPage();
            break;

        case PageId::LAT_LON:
            createLatLonPage();
            break;

        case PageId::NAV_DATA:
            createNavDataPage();
            break;

        case PageId::STATUS:
            createStatusPage();
            break;

        case PageId::SELECT_WPT:
            createSelectWaypointPage();
            break;
    }

    currentPage = page;
    redrawRequested = false;
}

// =====================================================
// BOARD SETUP
// =====================================================

void setup()
{
    Serial.begin(115200);

    Board *board =
        new Board();

    board->init();

#if LVGL_PORT_AVOID_TEARING_MODE
    auto lcd =
        board->getLCD();

    lcd->configFrameBufferNumber(
        LVGL_PORT_DISP_BUFFER_NUM
    );

#if ESP_PANEL_DRIVERS_BUS_ENABLE_RGB && CONFIG_IDF_TARGET_ESP32S3
    auto lcdBus =
        lcd->getBus();

    if (
        lcdBus->getBasicAttributes().type ==
        ESP_PANEL_BUS_TYPE_RGB
    ) {
        static_cast<BusRGB *>(lcdBus)
            ->configRGB_BounceBufferSize(
                lcd->getFrameWidth() * 10
            );
    }
#endif
#endif

    assert(
        board->begin()
    );

    sensorEspNowReady = setupSensorEspNow();

    Serial.print("ESP-NOW SENSOR LINK: ");
    Serial.println(sensorEspNowReady ? "READY" : "FAILED");

    Serial.print("ESP-NOW channel: ");
    Serial.println(SENSOR_ESPNOW_CHANNEL);

    Serial.print("Waveshare Wi-Fi MAC: ");
    Serial.println(WiFi.macAddress());

    initializeSdNavData(board);

    lvgl_port_init(
        board->getLCD(),
        board->getTouch()
    );

    rebuildRouteLegs();

    lvgl_port_lock(-1);

    renderPage(
        PageId::CDU_MENU
    );

    lvgl_port_unlock();
}

// =====================================================
// LOOP
// =====================================================

void loop()
{
    serviceSensorBus();
    serviceSensorPageRefresh();

    if (
        redrawRequested &&
        keyboardOverlay == nullptr &&
        millis() - keyboardClosedAtMs > 100
    ) {
        lvgl_port_lock(-1);

        renderPage(
            requestedPage
        );

        lvgl_port_unlock();
    }

    delay(10);
}