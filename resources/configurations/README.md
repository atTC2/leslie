# Sample Starting States

The following can be copied and pasted into the `"starting_state"` field in `leslie.json` to begin the system at various initial states.

## AT_HOME (default)
```json
{
    "id": "AT_HOME",
    "data": {}
}
```

## LISTENING\_FOR_TABLE
```json
{
    "id": "LISTENING_FOR_TABLE",
    "data": {
        "current_owner": "Tom"
    }
}
```

## MOVE\_TO_TABLE
```json
{
    "id": "MOVE_TO_TABLE",
    "data": {
        "tableID": 0,
        "current_owner": "Tom",
        "friend": null
    }
}
```

## AT_TABLE
```json
{
    "id": "AT_TABLE",
    "data": {
        "current_owner": "Tom",
        "friend": null
    }
}
```

## LOCKED\_AND_WAITING
```json
{
    "id": "LOCKED_AND_WAITING",
    "data": {
        "current_owner": "Tom",
        "friend": null
    }
}
```

## ALARM
```json
{
    "id": "ALARM",
    "data": {
        "current_owner": "Tom",
        "friend": null
    }
}
```

## ALARM_REPORT
```json
{
    "id": "ALARM_REPORT",
    "data": {
        "notify_owner": "Tom"
    }
}
```

## MOVE\_TO_HOME
```json
{
    "id": "MOVE_TO_HOME",
    "data": {}
}
```
